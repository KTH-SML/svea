#!/usr/bin/env python3

import json
import os
import re
import struct
import time
from importlib import import_module
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.set_message import set_message_fields
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes, PackageNotFoundError
from ament_index_python.resources import get_resources

from mavros_msgs.msg import Mavlink, Tunnel
from std_msgs.msg import String


_BUILTIN_SIZES: Dict[str, int] = {
    "int8": 1,
    "uint8": 1,
    "bool": 1,
    "char": 1,
    "int16": 2,
    "uint16": 2,
    "int32": 4,
    "uint32": 4,
    "float32": 4,
    "int64": 8,
    "uint64": 8,
    "float64": 8,
}

_BUILTIN_STRUCT_FMT: Dict[str, str] = {
    "int8": "b",
    "uint8": "B",
    "bool": "?",
    "char": "c",
    "int16": "h",
    "uint16": "H",
    "int32": "i",
    "uint32": "I",
    "float32": "f",
    "int64": "q",
    "uint64": "Q",
    "float64": "d",
}

_TOPICS_TOKEN = "# TOPICS "
_TYPE_RE = re.compile(r"^([A-Za-z0-9_/]+)(?:\[(\d+)\])?$")


@dataclass(frozen=True)
class MsgField:
    name: str
    type_token: str
    base_type: str
    is_builtin: bool
    array_len: int


@dataclass(frozen=True)
class LayoutEntry:
    kind: str  # "pad", "builtin", "nested"
    pad_bytes: int = 0
    field: Optional[MsgField] = None
    nested_type: Optional[str] = None


@dataclass(frozen=True)
class UorbSchema:
    type_name: str
    topic_names: tuple[str, ...]
    source_path: str
    fields_declared: tuple[MsgField, ...]
    layout: tuple[LayoutEntry, ...]
    end_padding: int
    size_no_padding: int
    size_padded: int
    message_hash: int


@dataclass
class PendingFrame:
    total_len: int
    message_hash: int
    next_offset: int
    payload: bytearray
    updated_at: float


class UorbSchemaRegistry:
    def __init__(self, msg_dirs: list[Path]) -> None:
        self._msg_dirs = msg_dirs
        self._raw_types: dict[str, dict[str, Any]] = {}
        self._schemas_by_type: dict[str, UorbSchema] = {}
        self._topic_to_schema: dict[str, UorbSchema] = {}
        self._hash_str_cache: dict[str, str] = {}

        self._load_raw_types()
        self._build_all_schemas()
        self._build_topic_map()

    @property
    def schema_count(self) -> int:
        return len(self._schemas_by_type)

    @property
    def topic_count(self) -> int:
        return len(self._topic_to_schema)

    @property
    def schema_types(self) -> tuple[str, ...]:
        return tuple(sorted(self._schemas_by_type.keys()))

    def has_topic(self, topic_name: str) -> bool:
        return topic_name in self._topic_to_schema

    def get_schema(self, topic_name: str) -> UorbSchema:
        schema = self._topic_to_schema.get(topic_name)
        if schema is None:
            raise KeyError(f"Unknown uORB topic: {topic_name}")
        return schema

    def get_schema_by_type(self, type_name: str) -> UorbSchema:
        schema = self._schemas_by_type.get(type_name)
        if schema is None:
            raise KeyError(f"Unknown uORB message type: {type_name}")
        return schema

    def decode_payload(
        self, topic_name: str, payload: bytes, expected_hash: int = 0
    ) -> dict[str, Any]:
        schema = self.get_schema(topic_name)

        if expected_hash and expected_hash != schema.message_hash:
            raise ValueError(
                f"Message hash mismatch for topic '{topic_name}': "
                f"expected 0x{expected_hash:08X}, schema has 0x{schema.message_hash:08X}"
            )

        include_end_padding = False
        payload_len = len(payload)

        if payload_len == schema.size_no_padding:
            include_end_padding = False
        elif payload_len == schema.size_padded:
            include_end_padding = True
        else:
            raise ValueError(
                f"Payload size mismatch for topic '{topic_name}': "
                f"got {payload_len}, expected {schema.size_no_padding} (no pad) "
                f"or {schema.size_padded} (padded)"
            )

        decoded, cursor = self._decode_schema_instance(
            schema, payload, 0, include_end_padding=include_end_padding
        )

        if cursor != payload_len:
            raise ValueError(
                f"Decode cursor mismatch for topic '{topic_name}': cursor={cursor}, payload_len={payload_len}"
            )

        return decoded

    def _load_raw_types(self) -> None:
        all_msg_files: list[Path] = []
        for msg_dir in self._msg_dirs:
            if not msg_dir.exists() or not msg_dir.is_dir():
                continue
            
            msg_files = sorted(msg_dir.glob("*.msg"))
            if not msg_files:
                msg_files = sorted(msg_dir.rglob("*.msg"))
            all_msg_files.extend(msg_files)

        if not all_msg_files:
            raise RuntimeError(f"No .msg files found in any search path: {self._msg_dirs}")

        for msg_path in all_msg_files:
            if not msg_path.is_file():
                continue

            type_name = msg_path.stem
            if type_name in self._raw_types:
                # If we have a duplicate, we keep the first one found (respects search path order)
                continue

            try:
                topic_names, fields = self._parse_msg_file(msg_path)
                self._raw_types[type_name] = {
                    "source_path": str(msg_path),
                    "topic_names": tuple(topic_names),
                    "fields": tuple(fields),
                }
            except Exception:
                # Skip invalid files gracefully
                continue

    def _build_all_schemas(self) -> None:
        for type_name in sorted(self._raw_types.keys()):
            self._build_schema(type_name, stack=())

    def _build_topic_map(self) -> None:
        for schema in self._schemas_by_type.values():
            for topic_name in schema.topic_names:
                existing = self._topic_to_schema.get(topic_name)
                if existing is not None and existing.type_name != schema.type_name:
                    raise RuntimeError(
                        f"Topic name collision '{topic_name}' between "
                        f"'{existing.type_name}' and '{schema.type_name}'"
                    )
                self._topic_to_schema[topic_name] = schema

    def _build_schema(self, type_name: str, stack: tuple[str, ...]) -> UorbSchema:
        cached = self._schemas_by_type.get(type_name)
        if cached is not None:
            return cached

        raw = self._raw_types.get(type_name)
        if raw is None:
            raise RuntimeError(f"Unknown message type referenced by field: {type_name}")

        if type_name in stack:
            cycle = " -> ".join((*stack, type_name))
            raise RuntimeError(f"Cyclic message dependency detected: {cycle}")

        fields_declared = list(raw["fields"])
        fields_sorted = sorted(fields_declared, key=self._field_sort_key, reverse=True)

        layout: list[LayoutEntry] = []
        offset = 0
        next_stack = (*stack, type_name)

        for field in fields_sorted:
            if field.is_builtin:
                builtin_size = _BUILTIN_SIZES[field.base_type]
                offset += builtin_size * field.array_len
                layout.append(LayoutEntry(kind="builtin", field=field))
                continue

            pad = (8 - (offset % 8)) % 8
            if pad:
                layout.append(LayoutEntry(kind="pad", pad_bytes=pad))
                offset += pad

            nested_type = self._resolve_type_name(field.base_type)
            nested_schema = self._build_schema(nested_type, stack=next_stack)
            offset += nested_schema.size_padded * field.array_len
            layout.append(
                LayoutEntry(kind="nested", field=field, nested_type=nested_type)
            )

        end_padding = (8 - (offset % 8)) % 8
        size_no_padding = offset
        size_padded = offset + end_padding
        message_hash = self._hash_32_fnv1a(self._message_fields_str(type_name))

        schema = UorbSchema(
            type_name=type_name,
            topic_names=raw["topic_names"],
            source_path=raw["source_path"],
            fields_declared=tuple(fields_declared),
            layout=tuple(layout),
            end_padding=end_padding,
            size_no_padding=size_no_padding,
            size_padded=size_padded,
            message_hash=message_hash,
        )
        self._schemas_by_type[type_name] = schema
        return schema

    def _message_fields_str(self, type_name: str) -> str:
        cached = self._hash_str_cache.get(type_name)
        if cached is not None:
            return cached

        raw = self._raw_types.get(type_name)
        if raw is None:
            raise RuntimeError(
                f"Cannot compute message hash for unknown type: {type_name}"
            )

        parts: list[str] = []
        for field in raw["fields"]:
            type_name_for_hash = field.type_token
            if "/" in type_name_for_hash:
                type_name_for_hash = type_name_for_hash.split("/", 1)[1]

            parts.append(f"{type_name_for_hash} {field.name}\n")

            if not field.is_builtin:
                nested_type = self._resolve_type_name(field.base_type)
                parts.append(self._message_fields_str(nested_type))

        combined = "".join(parts)
        self._hash_str_cache[type_name] = combined
        return combined

    def _decode_schema_instance(
        self,
        schema: UorbSchema,
        payload: bytes,
        cursor: int,
        *,
        include_end_padding: bool,
    ) -> tuple[dict[str, Any], int]:
        out: dict[str, Any] = {}

        for entry in schema.layout:
            if entry.kind == "pad":
                cursor = self._advance(cursor, entry.pad_bytes, len(payload), "padding")
                continue

            if entry.kind == "builtin":
                assert entry.field is not None
                value, cursor = self._decode_builtin(entry.field, payload, cursor)
                out[entry.field.name] = value
                continue

            if entry.kind == "nested":
                assert entry.field is not None
                assert entry.nested_type is not None
                nested_schema = self._schemas_by_type[entry.nested_type]
                if entry.field.array_len == 1:
                    value, cursor = self._decode_schema_instance(
                        nested_schema, payload, cursor, include_end_padding=True
                    )
                else:
                    value = []
                    for _ in range(entry.field.array_len):
                        element, cursor = self._decode_schema_instance(
                            nested_schema, payload, cursor, include_end_padding=True
                        )
                        value.append(element)

                out[entry.field.name] = value
                continue

            raise RuntimeError(f"Unexpected layout entry kind: {entry.kind}")

        if include_end_padding:
            cursor = self._advance(
                cursor,
                schema.end_padding,
                len(payload),
                f"{schema.type_name} end padding",
            )

        return out, cursor

    def _decode_builtin(
        self, field: MsgField, payload: bytes, cursor: int
    ) -> tuple[Any, int]:
        item_size = _BUILTIN_SIZES[field.base_type]
        count = field.array_len
        total_size = item_size * count
        cursor = self._advance(
            cursor, total_size, len(payload), f"field '{field.name}'"
        )
        start = cursor - total_size
        field_bytes = payload[start:cursor]

        if field.base_type == "char":
            if count == 1:
                char_val = field_bytes[0]
                return (chr(char_val) if char_val else ""), cursor

            return field_bytes.split(b"\x00", 1)[0].decode(
                "utf-8", errors="replace"
            ), cursor

        fmt = _BUILTIN_STRUCT_FMT[field.base_type]
        if count == 1:
            value = struct.unpack_from("<" + fmt, field_bytes, 0)[0]
            if field.base_type == "bool":
                value = bool(value)
            return value, cursor

        values = list(struct.unpack_from("<" + str(count) + fmt, field_bytes, 0))
        if field.base_type == "bool":
            values = [bool(v) for v in values]
        return values, cursor

    @staticmethod
    def _advance(cursor: int, delta: int, total_len: int, what: str) -> int:
        next_cursor = cursor + delta
        if next_cursor > total_len:
            raise ValueError(
                f"Out-of-bounds decode while reading {what}: cursor={cursor}, delta={delta}, total={total_len}"
            )
        return next_cursor

    @staticmethod
    def _hash_32_fnv1a(data: str) -> int:
        hash_val = 0x811C9DC5
        prime = 0x01000193
        for ch in data:
            hash_val ^= ord(ch)
            hash_val = (hash_val * prime) & 0xFFFFFFFF
        return hash_val

    @staticmethod
    def _to_snake_case(name: str) -> str:
        return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()

    @staticmethod
    def _field_sort_key(field: MsgField) -> int:
        if not field.is_builtin:
            return 0
        return _BUILTIN_SIZES[field.base_type]

    def _resolve_type_name(self, maybe_name: str) -> str:
        direct = maybe_name.split("/")[-1]
        if direct in self._raw_types:
            return direct

        snake_to_pascal = "".join(p.capitalize() for p in direct.split("_") if p)
        if snake_to_pascal in self._raw_types:
            return snake_to_pascal

        raise RuntimeError(
            f"Unknown nested message type referenced in .msg definitions: {maybe_name}"
        )

    def _parse_msg_file(self, msg_path: Path) -> tuple[list[str], list[MsgField]]:
        topic_names: list[str] = []
        fields: list[MsgField] = []

        for raw_line in msg_path.read_text(encoding="utf-8").splitlines():
            stripped = raw_line.strip()

            if stripped.startswith(_TOPICS_TOKEN):
                raw_topics = stripped[len(_TOPICS_TOKEN) :].split()
                for topic in raw_topics:
                    topic_snake = self._to_snake_case(topic)
                    if topic_snake not in topic_names:
                        topic_names.append(topic_snake)
                continue

            code = raw_line.split("#", 1)[0].strip()
            if not code:
                continue

            if "=" in code:
                # Constant definition
                continue

            parts = code.split()
            if len(parts) != 2:
                raise RuntimeError(
                    f"Invalid field definition in {msg_path}: '{raw_line}'"
                )

            type_token, field_name = parts
            base_type, array_len = self._parse_type_token(type_token, msg_path)
            base_type_direct = base_type.split("/")[-1]
            is_builtin = base_type_direct in _BUILTIN_SIZES

            fields.append(
                MsgField(
                    name=field_name,
                    type_token=type_token,
                    base_type=base_type_direct,
                    is_builtin=is_builtin,
                    array_len=array_len,
                )
            )

        if not fields:
            raise RuntimeError(f"Message file has no fields: {msg_path}")

        if not topic_names:
            topic_names.append(self._to_snake_case(msg_path.stem))

        return topic_names, fields

    @staticmethod
    def _parse_type_token(type_token: str, msg_path: Path) -> tuple[str, int]:
        match = _TYPE_RE.match(type_token)
        if not match:
            raise RuntimeError(f"Invalid type token '{type_token}' in {msg_path}")

        base_type = match.group(1)
        array_len_str = match.group(2)
        array_len = int(array_len_str) if array_len_str is not None else 1
        if array_len < 1:
            raise RuntimeError(
                f"Invalid array length in type token '{type_token}' in {msg_path}"
            )
        return base_type, array_len


class Px4UorbTunnelNode(Node):
    TUNNEL_MSG_ID = 385
    UORB_TUNNEL_PAYLOAD_TYPE = 0xE001

    # MAVROS payload64 exposes MAVLink payload in wire field order for TUNNEL:
    # payload_type (2), target_system (1), target_component (1), payload_length (1), payload[128]
    TUNNEL_MIN_WIRE_LEN = 5
    TUNNEL_OFFSET_PAYLOAD_TYPE = 0
    TUNNEL_OFFSET_PAYLOAD_LEN = 4
    TUNNEL_OFFSET_PAYLOAD = 5

    # Legacy protocol frame (existing SVEA stream format)
    LEGACY_PROTOCOL_VERSION = 1
    LEGACY_FRAME_HEADER_LEN = 5

    # Generic protocol frame (topic name + hash + fragmentation)
    GENERIC_PROTOCOL_VERSION = 2
    GENERIC_FRAME_BASE_HEADER_LEN = 14

    def __init__(self) -> None:
        super().__init__("px4_uorb_tunnel")

        self.declare_parameter("mavlink_topic", "/mavros/tunnel/out")
        self.declare_parameter("payload_type", self.UORB_TUNNEL_PAYLOAD_TYPE)
        self.declare_parameter("ros_msg_packages", "px4_msgs,svea_msgs")
        self.declare_parameter("legacy_topic_map", "1:power_monitor,2:gpio_in")
        self.declare_parameter("fragment_timeout_sec", 2.0)
        self.declare_parameter("override_msg_dir", "")

        self._mavlink_topic = str(self.get_parameter("mavlink_topic").value)
        self._payload_type = int(self.get_parameter("payload_type").value)
        self._legacy_topic_map_raw = str(self.get_parameter("legacy_topic_map").value)
        self._override_msg_dir = str(self.get_parameter("override_msg_dir").value).strip()
        self._fragment_timeout_sec = float(
            self.get_parameter("fragment_timeout_sec").value
        )

        # Automatic discovery of message-providing packages
        self._ros_msg_packages = self._parse_ros_msg_packages(
            self.get_parameter("ros_msg_packages").value, "px4_msgs"
        )
        self._msg_dirs = self._discover_all_msg_dirs(list(self._ros_msg_packages), self._override_msg_dir)

        self._schema_registry = UorbSchemaRegistry(self._msg_dirs)

        # Cache for ROS message modules
        self._ros_msg_modules: dict[str, Any] = {}
        for pkg in self._ros_msg_packages:
            try:
                self._ros_msg_modules[pkg] = import_module(f"{pkg}.msg")
            except (ImportError, ModuleNotFoundError):
                continue

        self._ros_msg_cls_cache: dict[str, Any] = {}
        self._legacy_topic_map = self._parse_legacy_topic_map(
            self._legacy_topic_map_raw
        )
        self._pending_frames: dict[tuple[str, int, int, int], PendingFrame] = {}
        self._topic_publishers_latest: dict[str, Any] = {}
        self._topic_publishers_instance: dict[tuple[str, int], Any] = {}

        self._frame_pub = self.create_publisher(String, "/px4/uorb_tunnel/frame", 100)
        self._validate_ros_message_compatibility()

        # Telemetry tracking
        self._tunnel_seen = 0
        self._tunnel_decoded = 0
        self._tunnel_dropped = 0
        self._payload_type_mismatch = 0
        self._decode_errors = 0
        self._unknown_protocol = 0
        self._reassembly_errors = 0
        
        # Track unknown topics so we only warn once per topic instead of spamming
        self._unknown_topics: set[str] = set()
        self._decode_error_logs = 0

        mavlink_qos = QoSProfile(depth=100)
        mavlink_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        mavlink_qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(
            Tunnel, self._mavlink_topic, self._on_tunnel, mavlink_qos
        )

        self.get_logger().info(
            f"PX4_UORB_TUNNEL active "
            f"(topic={self._mavlink_topic}, payload_type=0x{self._payload_type:04X}, "
            f"msg_dirs={[str(d) for d in self._msg_dirs]}, "
            f"packages={len(self._ros_msg_modules)}, "
            f"schemas={self._schema_registry.schema_count}, "
            f"topics={self._schema_registry.topic_count})"
        )

    def _log_decode_exception(self, context: str, exc: Exception) -> None:
        if self._decode_error_logs < 20:
            self.get_logger().error(f"{context}: {type(exc).__name__}: {exc}")
            self._decode_error_logs += 1

    @staticmethod
    def _publish_json(pub, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        pub.publish(msg)

    @staticmethod
    def _discover_all_msg_dirs(package_names: list[str], override_dir: str = "") -> list[Path]:
        dirs: list[Path] = []
        
        if override_dir:
            dirs.append(Path(override_dir))

        env_path = os.environ.get("PX4_MSG_DIR", "").strip()
        if env_path:
            dirs.append(Path(env_path))

        # Search for .msg files in all discovered message packages
        for pkg in package_names:
            try:
                share_dir = get_package_share_directory(pkg)
                msg_dir = Path(share_dir) / "msg"
                if msg_dir.is_dir():
                    dirs.append(msg_dir)
                else:
                    dirs.append(Path(share_dir))
            except PackageNotFoundError:
                continue

        # Deduplicate and keep existing
        seen = set()
        unique_dirs = []
        for d in dirs:
            if d not in seen and d.is_dir():
                unique_dirs.append(d)
                seen.add(d)
        
        return unique_dirs

    @staticmethod
    def _parse_ros_msg_packages(raw_packages: str, fallback_package: str) -> tuple[str, ...]:
        packages: list[str] = []

        if raw_packages:
            for token in raw_packages.split(","):
                pkg = token.strip()
                if pkg and pkg not in packages:
                    packages.append(pkg)

        if not packages and fallback_package:
            packages.append(fallback_package)

        return tuple(packages)

    def _get_ros_msg_cls(self, schema: UorbSchema):
        msg_cls = self._ros_msg_cls_cache.get(schema.type_name)
        if msg_cls is not None:
            return msg_cls

        for pkg, msg_module in self._ros_msg_modules.items():
            msg_cls = getattr(msg_module, schema.type_name, None)
            if msg_cls is not None:
                self._ros_msg_cls_cache[schema.type_name] = msg_cls
                return msg_cls

        raise RuntimeError(
            f"Missing ROS message class for uORB topic(s): {', '.join(schema.topic_names)}. "
            f"Searched all {len(self._ros_msg_modules)} discovered message packages."
        )

    def _validate_ros_message_compatibility(self) -> None:
        for type_name in self._schema_registry.schema_types:
            schema = self._schema_registry.get_schema_by_type(type_name)
            self._get_ros_msg_cls(schema)

    def _topic_latest_publisher(self, topic_name: str, msg_cls):
        pub = self._topic_publishers_latest.get(topic_name)
        if pub is not None:
            return pub

        ros_topic = f"/px4/uorb/{topic_name}"
        pub = self.create_publisher(msg_cls, ros_topic, 20)
        self._topic_publishers_latest[topic_name] = pub
        return pub

    def _topic_instance_publisher(self, topic_name: str, instance: int, msg_cls):
        key = (topic_name, instance)
        pub = self._topic_publishers_instance.get(key)
        if pub is not None:
            return pub

        ros_topic = f"/px4/uorb/{topic_name}/instance_{instance}"
        pub = self.create_publisher(msg_cls, ros_topic, 20)
        self._topic_publishers_instance[key] = pub
        return pub

    @staticmethod
    def _parse_legacy_topic_map(raw_map: str) -> dict[int, str]:
        out: dict[int, str] = {}
        if not raw_map.strip():
            return out

        for entry in raw_map.split(","):
            item = entry.strip()
            if not item:
                continue

            if ":" not in item:
                raise RuntimeError(
                    f"Invalid legacy_topic_map entry '{item}', expected '<id>:<topic>'"
                )

            left, right = item.split(":", 1)
            topic_id = int(left.strip(), 10)
            topic_name = right.strip()
            if topic_id < 0 or topic_id > 255:
                raise RuntimeError(
                    f"Invalid legacy topic id '{topic_id}', must be in [0,255]"
                )
            if not topic_name:
                raise RuntimeError(f"Invalid legacy topic name for id {topic_id}")
            out[topic_id] = topic_name

        return out

    def _decode_frame(self, payload: bytes) -> tuple[str, Optional[dict[str, Any]]]:
        if len(payload) < 1:
            return "error", None

        version = int(payload[0])

        if version == self.LEGACY_PROTOCOL_VERSION:
            frame = self._decode_legacy_frame(payload)
            if frame is None:
                return "error", None
            return "complete", frame

        if version == self.GENERIC_PROTOCOL_VERSION:
            status, frame = self._decode_generic_frame(payload)
            return status, frame

        self._unknown_protocol += 1
        return "error", None

    def _decode_legacy_frame(self, payload: bytes) -> Optional[dict[str, Any]]:
        if len(payload) < self.LEGACY_FRAME_HEADER_LEN:
            return None

        version, topic_id, instance, sequence, topic_payload_len = struct.unpack_from(
            "<BBBBB", payload, 0
        )
        if int(version) != self.LEGACY_PROTOCOL_VERSION:
            return None

        expected_len = self.LEGACY_FRAME_HEADER_LEN + int(topic_payload_len)
        if len(payload) != expected_len:
            return None

        topic_name = self._legacy_topic_map.get(int(topic_id))
        if topic_name is None:
            self._unknown_topic += 1
            return {
                "protocol_version": self.LEGACY_PROTOCOL_VERSION,
                "topic_id": int(topic_id),
                "topic_name": None,
                "instance": int(instance),
                "sequence": int(sequence),
                "message_hash": 0,
                "payload": payload[self.LEGACY_FRAME_HEADER_LEN :],
            }

        return {
            "protocol_version": self.LEGACY_PROTOCOL_VERSION,
            "topic_id": int(topic_id),
            "topic_name": topic_name,
            "instance": int(instance),
            "sequence": int(sequence),
            "message_hash": 0,
            "payload": payload[self.LEGACY_FRAME_HEADER_LEN :],
        }

    def _decode_generic_frame(
        self, payload: bytes
    ) -> tuple[str, Optional[dict[str, Any]]]:
        if len(payload) < self.GENERIC_FRAME_BASE_HEADER_LEN:
            return "error", None

        version, flags, instance, sequence = struct.unpack_from("<BBBB", payload, 0)
        if int(version) != self.GENERIC_PROTOCOL_VERSION:
            return "error", None

        message_hash = int(struct.unpack_from("<I", payload, 4)[0])
        topic_name_len = int(struct.unpack_from("<H", payload, 8)[0])
        total_len = int(struct.unpack_from("<H", payload, 10)[0])
        fragment_offset = int(struct.unpack_from("<H", payload, 12)[0])

        header_len = self.GENERIC_FRAME_BASE_HEADER_LEN + topic_name_len
        if topic_name_len == 0 or header_len > len(payload):
            return "error", None
        if total_len == 0:
            return "error", None

        topic_name_bytes = payload[self.GENERIC_FRAME_BASE_HEADER_LEN : header_len]
        try:
            topic_name = topic_name_bytes.decode("ascii")
        except UnicodeDecodeError:
            return "error", None

        fragment = payload[header_len:]
        if fragment_offset > total_len:
            return "error", None
        if fragment_offset + len(fragment) > total_len:
            return "error", None

        # Single-frame message
        if fragment_offset == 0 and len(fragment) == total_len:
            return (
                "complete",
                {
                    "protocol_version": self.GENERIC_PROTOCOL_VERSION,
                    "flags": int(flags),
                    "topic_id": None,
                    "topic_name": topic_name,
                    "instance": int(instance),
                    "sequence": int(sequence),
                    "message_hash": int(message_hash),
                    "payload": bytes(fragment),
                },
            )

        key = (topic_name, int(instance), int(sequence), int(message_hash))
        now = time.monotonic()
        self._drop_expired_pending(now)

        if fragment_offset == 0:
            pending = PendingFrame(
                total_len=total_len,
                message_hash=message_hash,
                next_offset=len(fragment),
                payload=bytearray(fragment),
                updated_at=now,
            )
            self._pending_frames[key] = pending
            return "pending", None

        pending = self._pending_frames.get(key)
        if pending is None:
            self._reassembly_errors += 1
            return "error", None

        if pending.total_len != total_len or pending.message_hash != message_hash:
            self._pending_frames.pop(key, None)
            self._reassembly_errors += 1
            return "error", None

        if fragment_offset != pending.next_offset:
            self._pending_frames.pop(key, None)
            self._reassembly_errors += 1
            return "error", None

        pending.payload.extend(fragment)
        pending.next_offset += len(fragment)
        pending.updated_at = now

        if pending.next_offset < pending.total_len:
            return "pending", None

        if pending.next_offset != pending.total_len:
            self._pending_frames.pop(key, None)
            self._reassembly_errors += 1
            return "error", None

        complete_payload = bytes(pending.payload)
        self._pending_frames.pop(key, None)

        return (
            "complete",
            {
                "protocol_version": self.GENERIC_PROTOCOL_VERSION,
                "flags": int(flags),
                "topic_id": None,
                "topic_name": topic_name,
                "instance": int(instance),
                "sequence": int(sequence),
                "message_hash": int(message_hash),
                "payload": complete_payload,
            },
        )

    def _drop_expired_pending(self, now: float) -> None:
        if self._fragment_timeout_sec <= 0:
            self._pending_frames.clear()
            return

        stale_keys = [
            key
            for key, pending in self._pending_frames.items()
            if (now - pending.updated_at) > self._fragment_timeout_sec
        ]
        for key in stale_keys:
            self._pending_frames.pop(key, None)

    def _on_tunnel(self, msg: Tunnel) -> None:
        self._tunnel_seen += 1

        payload_type = msg.payload_type
        if int(payload_type) != int(self._payload_type):
            self._payload_type_mismatch += 1
            self._tunnel_dropped += 1
            return

        payload = bytes(msg.payload[:msg.payload_length])
        
        frame_state, frame = self._decode_frame(payload)
        if frame_state == "pending":
            return

        if frame_state != "complete" or frame is None:
            self._decode_errors += 1
            self._tunnel_dropped += 1
            return

        body = frame["payload"]
        topic_name = frame["topic_name"]
        decoded_fields: Optional[dict[str, Any]] = None

        if topic_name is not None:
            try:
                decoded_fields = self._schema_registry.decode_payload(
                    topic_name=topic_name,
                    payload=body,
                    expected_hash=int(frame["message_hash"]),
                )
            except KeyError:
                if topic_name not in self._unknown_topics:
                    self._unknown_topics.add(topic_name)
                    self.get_logger().warn(
                        f"Received unknown uORB topic '{topic_name}' over tunnel. "
                        f"No .msg definition found in scanned packages. This topic will be ignored."
                    )
            except Exception as exc:
                self._log_decode_exception(
                    f"decode_payload failed for topic={topic_name}", exc
                )
                self._decode_errors += 1
                self._tunnel_dropped += 1
                return

        frame_payload = {
            "protocol_version": int(frame["protocol_version"]),
            "payload_type": int(payload_type),
            "topic_id": frame["topic_id"],
            "topic": topic_name,
            "instance": int(frame["instance"]),
            "sequence": int(frame["sequence"]),
            "message_hash": int(frame["message_hash"]),
            "payload_len": len(body),
            "payload_hex": body.hex(),
            "decoded": decoded_fields is not None,
        }
        self._publish_json(self._frame_pub, frame_payload)

        if topic_name is not None and decoded_fields is not None:
            try:
                schema = self._schema_registry.get_schema(topic_name)
                msg_cls = self._get_ros_msg_cls(schema)
                ros_msg = msg_cls()
                set_message_fields(ros_msg, decoded_fields)
                self._topic_latest_publisher(topic_name, msg_cls).publish(ros_msg)
                self._topic_instance_publisher(
                    topic_name, int(frame["instance"]), msg_cls
                ).publish(ros_msg)
            except Exception as exc:
                self._log_decode_exception(
                    f"ROS publish failed for topic={topic_name}", exc
                )
                self._decode_errors += 1
                self._tunnel_dropped += 1
                return

        self._tunnel_decoded += 1

        if (self._tunnel_seen % 1000) == 0:
            self.get_logger().debug(
                f"PX4_UORB_TUNNEL stats: seen={self._tunnel_seen} decoded={self._tunnel_decoded} "
                f"dropped={self._tunnel_dropped} payload_type_mismatch={self._payload_type_mismatch} "
                f"decode_errors={self._decode_errors} unknown_topics={len(self._unknown_topics)} "
                f"unknown_protocol={self._unknown_protocol} reassembly_errors={self._reassembly_errors}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4UorbTunnelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
