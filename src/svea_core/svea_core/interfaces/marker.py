#!/usr/bin/env python
from typing import Sequence

import tf_transformations as tf
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from visualization_msgs.msg import Marker
from PIL import ImageColor

from .. import rosonic as rx

qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

class ShowMarker(rx.NamedField):
    
    # {parent} will expand to name of interface
    _pub = rx.Publisher(Marker, '~/{parent}', qos_profile=qos_pubber)

    def place(
        self,
        position: Sequence[float] | None = None,
        orientation: Sequence[float] | None = None,
        frame_id: str = "map",
        color: str | Sequence[int] = 'white',
        shape: int = Marker.SPHERE,
        scale: float | Sequence[float] = 0.5,
        **kwds,
    ) -> None:
        
        if position is None:
            position = [0.0, 0.0, 0.0]
        elif len(position) == 2:
            position = [position[0], position[1], 0.0]
        elif len(position) != 3:
            raise ValueError("Position must be a sequence of three floats.")

        if orientation is None:
            orientation = [0.0, 0.0, 0.0, 1.0]
        elif len(orientation) == 3:
            x, y, z = tf.quaternion_from_euler(orientation[0], orientation[1], orientation[2])
            orientation = [x, y, z, 1.0]
        elif len(orientation) != 4:
            raise ValueError("Orientation must be a sequence of three (Euler) or four (quaternion) floats.")
        
        if isinstance(scale, (int, float)):
            scale = [scale] * 3
        elif len(scale) != 3:
            raise ValueError("Scale must be a float or a sequence of three floats.")

        r, g, b, a = self.parse_color(color)

        self.node.get_logger().debug(f"Placing marker at {position}")
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.type = shape
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2] if len(position) > 2 else 0.0
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        self._pub.publish(marker)

    @staticmethod
    def parse_color(input_color, default_alpha=1.0):
        """
        Parse color input and return in RGBA format.

        Args:
            input_color: Input color, can be:
                - Color name string (e.g., 'red', 'blue')
                - Hex string (e.g., '#FF0000', '#FF0000FF')
                - RGB/RGBA tuple (e.g., (255, 0, 0) or (1.0, 0.0, 0.0, 0.5))
            default_alpha: Default alpha value when input color has no alpha channel (0.0-1.0)
        
        Returns:
            RGBA tuple with all components in 0.0–1.0 range.
        """

        # If already a tuple or list
        if isinstance(input_color, (tuple, list)) and len(input_color) in (3, 4):
            if all(isinstance(c, int) and 0 <= c <= 255 for c in input_color[:3]):
                r, g, b = [c / 255.0 for c in input_color[:3]]
                a = input_color[3] / 255.0 if len(input_color) == 4 else default_alpha
                return r, g, b, a
            elif all(isinstance(c, float) and 0.0 <= c <= 1.0 for c in input_color[:3]):
                r, g, b = input_color[:3]
                a = input_color[3] if len(input_color) == 4 else default_alpha
                return r, g, b, a

        # If string input (color name or hex)
        if isinstance(input_color, str):
            try:
                # Pillow returns RGB or RGBA as integers 0–255
                rgb = ImageColor.getrgb(input_color)
                if len(rgb) == 3:
                    r, g, b = [c / 255.0 for c in rgb]
                    return r, g, b, default_alpha
                elif len(rgb) == 4:
                    r, g, b, a = [c / 255.0 for c in rgb]
                    return r, g, b, a
            except ValueError:
                pass  # unrecognized string

        # Fallback — return white
        return 1.0, 1.0, 1.0, 1.0
