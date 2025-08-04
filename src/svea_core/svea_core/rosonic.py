"""
<<<<<<< HEAD
<<<<<<< HEAD
# Declarative ROS 2 Nodes in Python
=======
# rosonic: Declarative ROS 2 Nodes in Python
>>>>>>> f6498d4 (update rosonic)

**Note:** This started as a fun project in ROS 1 to make nodes easier to write.
With ROS 2, I thought rosonic wouldn't be necessary. However, unsurprisingly,
ROS took one step forward and two steps back. So, here we are again.

**rosonic** is a minimalistic framework for writing ROS 2 nodes declaratively
in Python. It allows you to define *Parameters*, *Publishers*, *Subscribers*,
and *Timers* as simple **class fields**, abstracting away repetitive
boilerplate and focusing on **what resources your node needs**, not how to
create them.

This module provides:
- Declarative creation of ROS resources (parameters, publishers, subscribers, timers).
- Automatic resource registration and namespacing.
- Lifecycle hooks for initialization and shutdown.
- Clean syntax with minimal magic, built directly on top of `rclpy`.

## Why Rosonic?

While ROS 2 introduced improvements over ROS 1, it also made some common tasks
more verbose and procedural. Defining a simple node with parameters and
publishers often involves repetitive code and explicit setup in the `__init__`
method.

rosonic simplifies this by:
- Letting you declare your node's resources as class attributes.
- Handling the initialization lifecycle of resources automatically.
- Reducing boilerplate so you can focus on behavior, not plumbing.

## Key Concepts

### Resources
- `Resource` is the base class for all declarative entities (parameters,
  publishers, subscribers, timers) in a node, which is a resource itself.
- Resources are hierarchically registered to form a tree structure that mirrors
  the ROS graph namespace.

### Fields vs NamedFields
Rosonic distinguishes between **structural fields** (for grouping) and **named
fields** (which reserve a name in the graph):

| Type           | Purpose                                                  |
| -------------- | -------------------------------------------------------- |
| `Field`        | Structural container. Doesn't appear in graph names.     |
|                | Can have a name which children are scoped under, or      |
|                | flattened to grandparent if missing name.                |
| `NamedField`   | Named resource. Appears in graph names. Its children are |
|                | always nested under its name.                            |

### Naming Propagation (Flattening Logic)
The following diagram shows how Field and NamedField affect naming:

```
Example 1: Field with Name
MyNode (Node)                       --> /my_node
└── iface (Field, name="interface") --> /my_node/interface
    ├── param1 (Parameter)          --> /my_node/interface/param1
    └── pub1 (Publisher)            --> /my_node/interface/pub1

Example 2: Field without Name (Flattened. Does not reserve name.)
MyNode (Node)               --> /my_node
└── iface (Field, no name)
    ├── param1 (Parameter)  --> /my_node/param1
    └── pub1 (Publisher)    --> /my_node/pub1

Example 3: NamedField (Always reserves a name. Falls back to attribute name if
none given.)
MyNode (Node)                   --> /my_node
└── iface (NamedField, no name) --> /my_node/iface
    ├── param1 (Parameter)      --> /my_node/iface/param1
    └── pub1 (Publisher)        --> /my_node/iface/pub1
```

### Node Lifecycle
- You define a Node by inheriting from `rosonic.Node`.
- Implement `on_startup()` to define startup behavior (e.g., timers, dynamic
  resource handling).
- Optionally implement `on_shutdown()` for cleanup.
- Entry point via `Node.main()`, which initializes, spins, and shuts down the
  node gracefully.

## Example: Writing a Talker Node

```python
import rosonic as rx
from std_msgs.msg import String

class Talker(rx.Node):

    topic_name = rx.Parameter('chat')           # Declare a parameter
    talk_pub = rx.Publisher(String, topic_name) # Declare a publisher tied to the parameter

    @rx.Timer(1/5.0, autostart=False)
    def talk_loop(self):
        msg = String()
        msg.data = "Hello from rosonic!"
        self.talk_pub.publish(msg)

    def on_startup(self):

        # ... Initialization behavior

        self.talk_loop.reset() # Starts the loop

if __name__ == "__main__":
    Talker.main()
````

## Supported Resource Types

| Resource     | Description                                                                |
| ------------ | -------------------------------------------------------------------------- |
| `Parameter`  | Declarative node parameter. Can be used to dynamically assign topic names. |
| `Publisher`  | Declarative publisher. Accepts topic as string or Parameter.               |
| `Subscriber` | Declarative subscriber. Use as a decorator to assign callbacks.            |
| `Timer`      | Declarative timer. Use as a decorator for periodic callbacks.              |

## Installation

rosonic is a **single-file module**. No installation is required—simply place
`rosonic.py` in your project and import it:

```python
import rosonic as rx
```

Developed for ROS 2 Jazzy Jalisco.

## Design Philosophy

* **Minimalistic and Transparent**: No hidden layers of abstraction beyond standard `rclpy` concepts.
* **Declarative First**: Focus on describing the ROS graph structure in a clear, class-based manner.
* **Lifecycle-Aware**: Resources are initialized in the correct order, respecting dependencies (e.g., Parameters before Publishers that use them).
* **No Code Generation**: You write Python; rosonic handles registration and lifecycle hooks for you.

## Limitations & Future Work

* Currently supports only **Parameters**, **Publishers**, **Subscribers**, and **Timers**.
* Service and Action support could be added following the same declarative principles.
* Advanced lifecycle management (e.g., ComponentNode patterns) are out of scope for now.

## Author
<<<<<<< HEAD
=======
This started as a fun project in ROS 1 to make nodes easier to write. With 
ROS 2, I thought rosonoic wouldn't be necessary. However, unsurprisingly, ROS
took one step forward and two steps back. So, here we are again.
>>>>>>> 11eeed9 (Fundamental changes.)
=======
>>>>>>> f6498d4 (update rosonic)

Kaj Munhoz Arfvidsson
"""

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
from __future__ import annotations

=======
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
=======
>>>>>>> 17e7e32 (Fundamental changes.)
=======
>>>>>>> e1d0aaa (update rosonic)
from typing import TypeGuard

import rclpy                            # pyright: ignore[reportMissingImports]
from rclpy.node import Node as NodeBase # pyright: ignore[reportMissingImports]

__all__ = [
    'Resource',
    'Node',
    'Field',
    'NamedField',
    'Parameter',
    'Publisher',
    'Subscriber',
    'Timer',
]

class Resource:
    """
    Base class for all declarative resources in rosonic.

    A Resource represents a conceptual entity in the ROS 2 computation graph,
    such as a Parameter, Publisher, Subscriber, or Timer. Resources are
    organized into a hierarchical tree, which mirrors the graph namespace
    structure in ROS.

    This class handles:
    - Resource ownership (parent-child relations).
    - Name and namespace composition.
    - Registration into the resource tree.
    - Lifecycle hooks for startup and shutdown.

    Subclasses should implement resource-specific behavior (e.g., Parameters
    declare values, Publishers create publishers, etc.).

    ## Lifecycle Methods

    1. `__rosonic_register__`: Called to register the resource into the
       ownership hierarchy.
    2. `__rosonic_startup__`: Called when the node starts; triggers
       resource-specific setup through the hook `on_startup`.
    3. `__rosonic_shutdown__`: Called on shutdown for cleanup through the
       hook `on_shutdown`.

    ## Resource Lookup

    Resources maintain a recursive lookup dictionary mapping fully qualified
    names to the actual Resource instances. This allows for introspection of
    the resource tree if needed.

    - `__rosonic_name__`: The local name assigned to this resource.
    - `__rosonic_relname__`: The relative path of this resource within the
      resource tree (i.e. without the root resource name, which is often fully
      qualified). Can be fully qualified if an ancestor other than the root
      provides an absolute name.
    - `__rosonic_fullname__`: The fully qualified graph name of this resource

    Note: Names starting with `/` or `~` are treated as absolute names.
    """

    __rosonic_name__: str | None                    = None
    __rosonic_node__: 'NodeBase | None'             = None
    __rosonic_owner__: 'Resource | None'            = None
    __rosonic_resources__: tuple['Resource', ...]   = ()
<<<<<<< HEAD
    __rosonic_started__: bool                       = False
=======
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))

    # Class property. For Field resources
    __rosonic_preregistered__: tuple['Resource', ...] = ()

    def __init__(self, *, name: str | None = None):
        """
        Initializes a new Resource.

        Args:
            name (str | None): Optional name for the resource. If omitted, the
            resource may receive a name later via attribute binding (for
            NamedFields), or remain unnamed (for structural Fields).
        """
        self.__rosonic_name__ = name

    def __rosonic_register__(self, owner: 'Resource', *, name: str | None = None):
        """
        Internal method to register this resource under a parent resource (owner).

        This method assigns ownership, resolves naming inheritance, and
        recursively registers all child resources.

        Args:
            owner (Resource): The parent resource that owns this one.
            name (str | None): Optional name override for this resource.
            ns (str | None): Optional namespace override for this resource.

        Raises:
            Exception: If naming conflicts or invalid ownership hierarchy is
            detected.
        """

        if name is not None:
            if self.__rosonic_name__ is None:
                self.__rosonic_name__ = name
            else:
                raise Exception(f"Resource '{self}' already has a name, '{self.__rosonic_name__}'")

        if owner is self:
            assert self.__rosonic_name__ is not None, f"Root resource '{self}' must have a name"
        else:
<<<<<<< HEAD
            assert owner._is_registered(), f"Resource '{owner}' owning '{self}' is not registered"
=======
            assert _is_registered(owner), f"Resource '{owner}' owning '{self}' is not registered"
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))

            # The following check only makes sense for non-root, named resources
            if self.__rosonic_name__ is not None:
                fullname = f"{owner.__rosonic_fullname__}/{self.__rosonic_name__}"
                assert fullname not in self.__rosonic_lookup__, \
                    f"Resource '{fullname}' already registered"

            owner.__rosonic_resources__ += (self,)

        self.__rosonic_owner__ = owner # From this point, the resource (self) is registered
        
        for resource in self.__rosonic_preregistered__:
            resource.__rosonic_register__(self)

    @property
    def __rosonic_fullname__(self) -> str:
        """
        Computes the fully qualified graph name of this resource.

        Note: Takes on the parent's name if itself does not have a name.

        Returns:
            str: The fully qualified name for this resource.

        Raises:
            AssertionError: If the resource is not registered yet.
        """
<<<<<<< HEAD
        assert self._is_registered(), f"Resource '{self}' is not started"
=======
        assert _is_registered(self), f"Resource '{self}' is not started"
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))

        name = self.__rosonic_name__
        owner = self.__rosonic_owner__

        if name is None:
            return owner.__rosonic_fullname__
<<<<<<< HEAD
        elif self._is_root() or self._is_absolute_name(name):
=======
        elif _is_root(self) or _is_absolute_name(name):
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
            return name
        else:
            return f"{owner.__rosonic_fullname__}/{name}"

    @property
    def __rosonic_relname__(self) -> str:
        """
        Computes the relative name of this resource with respect to the root
        resource.

        Note: Takes on the parent's name if itself does not have a name.

        Returns:
            str: The relative name of this resource.

        Raises:
            AssertionError: If the resource is not registered yet.
        """
<<<<<<< HEAD
        assert self._is_registered(), f"Resource '{self}' is not started"
=======
        assert _is_registered(self), f"Resource '{self}' is not started"
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
        
        name = self.__rosonic_name__
        owner = self.__rosonic_owner__

        if name is None:
            return owner.__rosonic_relname__
<<<<<<< HEAD
        elif self._is_root():
            return ''
        elif self._is_absolute_name(name) or owner._is_root():
            return name
        else:
            parent = owner.__rosonic_relname__
            return name if parent == '' else f"{parent}/{name}"
    
=======
        elif _is_root(self):
            return ''
        elif _is_absolute_name(name) or _is_root(owner):
            return name
        else:
            return f"{owner.__rosonic_relname__}/{name}"

>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
    @property
    def __rosonic_lookup__(self) -> dict[str, 'Resource']:
        """
        Recursively builds a lookup dictionary mapping fully qualified names
        to all descendant resources in the resource tree.

        Returns:
            dict[str, Resource]: A dictionary of all child resources accessible
            by full name.
        """
        lookup: dict[str, 'Resource'] = {}
        for resource in self.__rosonic_resources__:
            if resource.__rosonic_name__ is not None:
                lookup |= {resource.__rosonic_fullname__: resource}
            lookup |= resource.__rosonic_lookup__
        return lookup

    def __rosonic_startup__(self, node: NodeBase) -> None:
        """
        Starts up this resource and all its children.

        This method is called by the owning node during startup.
        It initializes the resource by invoking `on_startup()`,
        and then propagates startup to all child resources.

        Args:
            node (rclpy.node.Node): The ROS 2 node instance this resource
            belongs to.

        Raises:
            AssertionError: If the resource is not properly registered before
            startup.
        """
<<<<<<< HEAD
        assert self._is_registered(), f"Resource '{self}' is not registered"
=======
        assert _is_registered(self), f"Resource '{self}' is not registered"
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))

        for resource in self.__rosonic_resources__:
            resource.__rosonic_startup__(node)

        self.__rosonic_node__ = node

        self.on_startup()
<<<<<<< HEAD

        self.__rosonic_started__ = True
=======
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
        
    def __rosonic_shutdown__(self, node: NodeBase) -> None:
        """
        Shuts down this resource and all its children.

        This method is called by the owning node during shutdown. It invokes
        `on_shutdown()` for cleanup, then recursively shuts down child
        resources.

        Args:
            node (rclpy.node.Node): The ROS 2 node instance this resource
            belongs to.

        Raises:
            AssertionError: If the resource is not started yet.
        """
<<<<<<< HEAD
        assert self._is_started(), f"Resource '{self}' is not started"
=======
        assert _is_started(self), f"Resource '{self}' is not started"
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))

        self.on_shutdown()

        self.__rosonic_node__ = None

        for resource in self.__rosonic_resources__:
            resource.__rosonic_shutdown__(node)
    
    def on_startup(self):
        """
        Hook for subclasses to implement resource-specific startup logic.

        This method is called after the resource has been properly registered
        and just as it starts handling ROS computation (e.g., declaring
        parameters, creating publishers). Override this method in subclasses as
        needed.
        """
        pass

    def on_shutdown(self):
        """
        Hook for subclasses to implement resource-specific shutdown logic.

        This method is called during node shutdown to allow resource-specific cleanup
        (e.g., destroying publishers, unsubscribing).
        Override this method in subclasses as needed.
        """
        pass

<<<<<<< HEAD
    def _is_absolute_name(self, name: str | None = None) -> bool:
        if name is None:
            name = self.__rosonic_name__
        return name.startswith('/') or name.startswith('~')

    def _is_registered(self) -> TypeGuard['_RegisteredResource']:
        return self.__rosonic_owner__ is not None

    def _is_started(self) -> TypeGuard['_StartedResource']:
        return self.__rosonic_started__

    def _is_root(self) -> bool:
        return self.__rosonic_owner__ is self

    def _get_root(self) -> Resource:
        assert self._is_registered(), f"Resource '{self}' is not registered"
        return (self if self._is_root() else
                self._get_root())

=======
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))

# Only for typing
class _RegisteredResource(Resource):

    __rosonic_owner__: Resource     # pyright: ignore[reportIncompatibleVariableOverride]


# Only for typing
class _StartedResource(_RegisteredResource):

    __rosonic_node__: NodeBase


<<<<<<< HEAD
=======
def _is_absolute_name(name: str) -> bool:
    return name.startswith('/') or name.startswith('~')

def _is_registered(resource: Resource) -> TypeGuard[_RegisteredResource]:
    return resource.__rosonic_owner__ is not None

def _is_started(resource: Resource) -> TypeGuard[_StartedResource]:
    return resource.__rosonic_node__ is not None

def _is_root(resource: Resource) -> bool:
    return resource.__rosonic_owner__ is resource

def _get_root(resource: Resource) -> Resource:
    assert _is_registered(resource), f"Resource '{resource}' is not registered"
    return (resource if _is_root(resource) else
            _get_root(resource))

>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
class Node(Resource, NodeBase):
=======
from typing import Optional
=======
from typing import TypeGuard
>>>>>>> f6498d4 (update rosonic)

import rclpy                            # pyright: ignore[reportMissingImports]
from rclpy.node import Node as NodeBase # pyright: ignore[reportMissingImports]

__all__ = [
    'Resource',
    'Node',
    'Field',
    'NamedField',
    'Parameter',
    'Publisher',
    'Subscriber',
    'Timer',
]

class Resource:
    """
    Base class for all declarative resources in rosonic.

    A Resource represents a conceptual entity in the ROS 2 computation graph,
    such as a Parameter, Publisher, Subscriber, or Timer. Resources are
    organized into a hierarchical tree, which mirrors the graph namespace
    structure in ROS.

    This class handles:
    - Resource ownership (parent-child relations).
    - Name and namespace composition.
    - Registration into the resource tree.
    - Lifecycle hooks for startup and shutdown.

    Subclasses should implement resource-specific behavior (e.g., Parameters
    declare values, Publishers create publishers, etc.).

    ## Lifecycle Methods

    1. `__rosonic_register__`: Called to register the resource into the
       ownership hierarchy.
    2. `__rosonic_startup__`: Called when the node starts; triggers
       resource-specific setup through the hook `on_startup`.
    3. `__rosonic_shutdown__`: Called on shutdown for cleanup through the
       hook `on_shutdown`.

    ## Resource Lookup

    Resources maintain a recursive lookup dictionary mapping fully qualified
    names to the actual Resource instances. This allows for introspection of
    the resource tree if needed.

    - `__rosonic_name__`: The local name assigned to this resource.
    - `__rosonic_relname__`: The relative path of this resource within the
      resource tree (i.e. without the root resource name, which is often fully
      qualified). Can be fully qualified if an ancestor other than the root
      provides an absolute name.
    - `__rosonic_fullname__`: The fully qualified graph name of this resource

    Note: Names starting with `/` or `~` are treated as absolute names.
    """

    __rosonic_name__: str | None                    = None
    __rosonic_node__: 'NodeBase | None'             = None
    __rosonic_owner__: 'Resource | None'            = None
    __rosonic_resources__: tuple['Resource', ...]   = ()

    # Class property. For Field resources
    __rosonic_preregistered__: tuple['Resource', ...] = ()

    def __init__(self, *, name: str | None = None):
        """
        Initializes a new Resource.

        Args:
            name (str | None): Optional name for the resource. If omitted, the
            resource may receive a name later via attribute binding (for
            NamedFields), or remain unnamed (for structural Fields).
        """
        self.__rosonic_name__ = name

    def __rosonic_register__(self, owner: 'Resource', *, name: str | None = None):
        """
        Internal method to register this resource under a parent resource (owner).

        This method assigns ownership, resolves naming inheritance, and
        recursively registers all child resources.

        Args:
            owner (Resource): The parent resource that owns this one.
            name (str | None): Optional name override for this resource.
            ns (str | None): Optional namespace override for this resource.

        Raises:
            Exception: If naming conflicts or invalid ownership hierarchy is
            detected.
        """
        
        match (self.__rosonic_name__, name):
            case (  None,   None):
                raise Exception(f"Resource '{self}' doesn't have a name.")
            case (  None, str(_)):
                self.__rosonic_name__ = name
            case (str(_),   None): pass
            case (str(_), str(_)):
                raise Exception(f"Resource '{self}' already has a name, '{self.__rosonic_name__}'")

        self.__rosonic_owner__ = owner
        fullname = self.__rosonic_fullname__
        assert fullname not in self.__rosonic_lookup__, \
            f"Resource '{fullname}' already registered"

        if owner is not self:
            assert _is_registered(owner), f"Resource '{owner}' owning '{self}' is not registered"
            owner.__rosonic_resources__ += (self,)
        
        for resource in self.__rosonic_preregistered__:
            resource.__rosonic_register__(self)

    @property
    def __rosonic_fullname__(self) -> str:
        """
        Computes the fully qualified graph name of this resource.

        Returns:
            str: The fully qualified name for this resource.

        Raises:
            AssertionError: If the resource is not registered yet.
        """
        assert _is_registered(self), f"Resource '{self}' is not started"

        name = self.__rosonic_name__
        if _is_absolute_name(name):
            return name

        owner = self.__rosonic_owner__
        root = (None if _is_root(self) else
                owner.__rosonic_fullname__)

        return (name if root is None else
                f"{root}/{name}")

    @property
    def __rosonic_relname__(self) -> str:
        """
        Computes the relative name of this resource with respect to the root
        resource.

        Returns:
            str: The relative name of this resource.

        Raises:
            AssertionError: If the resource is not registered yet.
        """
        assert _is_registered(self), f"Resource '{self}' is not started"
        
        name = self.__rosonic_name__
        owner = self.__rosonic_owner__

        if _is_absolute_name(name):
            return name
        if _is_root(self):
            return ''

        return (name if _is_root(owner) else
                f"{owner.__rosonic_relname__}/{name}")

    @property
    def __rosonic_lookup__(self) -> dict[str, 'Resource']:
        """
        Recursively builds a lookup dictionary mapping fully qualified names
        to all descendant resources in the resource tree.

        Returns:
            dict[str, Resource]: A dictionary of all child resources accessible
            by full name.
        """
        lookup: dict[str, 'Resource'] = {}
        for resource in self.__rosonic_resources__:
            lookup |= {resource.__rosonic_fullname__: resource}
            lookup |= resource.__rosonic_lookup__
        return lookup

    def __rosonic_startup__(self, node: NodeBase) -> None:
        """
        Starts up this resource and all its children.

        This method is called by the owning node during startup.
        It initializes the resource by invoking `on_startup()`,
        and then propagates startup to all child resources.

        Args:
            node (rclpy.node.Node): The ROS 2 node instance this resource
            belongs to.

        Raises:
            AssertionError: If the resource is not properly registered before
            startup.
        """
        assert _is_registered(self), f"Resource '{self}' is not registered"

        for resource in self.__rosonic_resources__:
            resource.__rosonic_startup__(node)

        self.__rosonic_node__ = node

        self.on_startup()
        
    def __rosonic_shutdown__(self, node: NodeBase) -> None:
        """
        Shuts down this resource and all its children.

        This method is called by the owning node during shutdown. It invokes
        `on_shutdown()` for cleanup, then recursively shuts down child
        resources.

        Args:
            node (rclpy.node.Node): The ROS 2 node instance this resource
            belongs to.

        Raises:
            AssertionError: If the resource is not started yet.
        """
        assert _is_started(self), f"Resource '{self}' is not started"

        self.on_shutdown()

        self.__rosonic_node__ = None

        for resource in self.__rosonic_resources__:
            resource.__rosonic_shutdown__(node)
    
    def on_startup(self):
        """
        Hook for subclasses to implement resource-specific startup logic.

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
class Parameter(Member):
    """
    Class to represent a parameter in a ROS 2 node.
    This class is used to define parameters for the node and provides
    functionality to set and get parameter values.
    """

    def __init__(self, *args, name=None, **kwds):
        """
        Initialize the parameter with a name and an optional default value.
        """
        self._name = name
        self._args = args
        self._kwds = kwds

    def __get__(self, instance, owner):
        """
        Get the value of the parameter.
        """
        if instance is None:
            return self

        if not self.__rosonic_active__:
            return self

        parameters = self.__rosonic_node__.__rosonic_parameters__
        
        return parameters[self._name].value
    
    def on_startup(self):
        """
        Called when the node is started.
        This method is used to declare the parameter in the node.
        """
        if self._name is None:
            self._name = self.__rosonic_name__
        
        node = self.__rosonic_node__
        parameters = node.__rosonic_parameters__

<<<<<<< HEAD
class Node(rclpy.node.Node):
>>>>>>> 11eeed9 (Fundamental changes.)
=======
        if self._name not in parameters:
            node.declare_parameter(self._name, *self._args, **self._kwds)
            parameters[self._name] = node.get_parameter(self._name)
    

class Publisher(Member):
    """
    Class to represent a publisher in a ROS 2 node.
    This class is used to define publishers for the node and provides
    functionality to publish messages.
    """

    def __init__(self, msg_type, topic, qos_profile=None):
        """
        Initialize the publisher with a topic, message type, and optional QoS profile.
        """
        self._topic = topic
        self._msg_type = msg_type
        self._qos_profile = qos_profile
        self._publisher = None

    def publish(self, msg):
        """
        Publish a message to the topic.
        Raise an exception if the publisher is not started.
        """
        if self._publisher is None:
            raise RuntimeError(f"Publisher for topic '{self._topic}' is not started yet.")
        self._publisher.publish(msg)

    def on_startup(self):
        """
        Called when the node is started.
        This method is used to create the publisher in the node.
        """
        node = self.__rosonic_node__
        msg_type = self._msg_type
        topic = self._topic
        qos_profile = self._qos_profile

        if isinstance(topic, Parameter):
            topic = self.__rosonic_node__.__rosonic_parameters__[topic._name].value

        if not isinstance(msg_type, type):
            raise RuntimeError(f"Message type must be a class, not {type(msg_type)}")
        if not isinstance(topic, str):
            raise RuntimeError(f"Topic name must be a string, not {type(topic)}")
        if not isinstance(qos_profile, rclpy.qos.QoSProfile):
            raise RuntimeError(f"QoS profile must be a QoSProfile, not {type(qos_profile)}")

        self._publisher = node.create_publisher(msg_type, topic, qos_profile)


class Subscriber(Member):
    """
    Class to represent a subscriber in a ROS 2 node.
    This class is used to define subscribers for the node and provides
    functionality to handle incoming messages via a callback.
    """

    def __init__(self, msg_type, topic, qos_profile=None):
        """
        Initialize the subscriber with a topic, message type, and optional QoS profile.
        """
        self._topic = topic
        self._msg_type = msg_type
        self._qos_profile = qos_profile
        self._subscriber = None
        self._callback = None

    def __call__(self, callback):
        """
        Use this subscriber as a decorator to set the callback function.
        """
        self._callback = callback
        return self

    def on_startup(self):
        """
        Called when the node is started.
        This method is used to create the subscriber in the node.
        """

        node = self.__rosonic_node__
        msg_type = self._msg_type
        topic = self._topic
        qos_profile = self._qos_profile

        if isinstance(topic, Parameter):
            topic = self.__rosonic_node__.__rosonic_parameters__[topic._name].value

        if not isinstance(msg_type, type):
            raise RuntimeError(f"Message type must be a class, not {type(msg_type)}")
        if not isinstance(topic, str):
            raise RuntimeError(f"Topic name must be a string, not {type(topic)}")
        if not isinstance(qos_profile, rclpy.qos.QoSProfile):
            raise RuntimeError(f"QoS profile must be a QoSProfile, not {type(qos_profile)}")

        if self._callback is None:
            raise RuntimeError(f"Callback for subscriber '{self._topic}' is not set.")

        _callback = self._callback
        _parent = self.__rosonic_parent__

        def callback(msg):
            return _callback(_parent, msg)

        self._subscriber = node.create_subscription(msg_type, topic, callback, qos_profile)
=======
    def on_startup(self) -> None: ...
>>>>>>> 24b9acb (minor structural changes to rosonic)

    def on_shutdown(self) -> None: ...

=======
>>>>>>> 217dc92 (05/12/2025 meeting update)
class Node(Member, rclpy.node.Node):
>>>>>>> 4b0286b (More work on simulator)
=======
class Node(Member, Node):
>>>>>>> 8b92c94 (added mpc control and example, but still in working progress)
=======
        This method is called after the resource has been properly registered
        and just as it starts handling ROS computation (e.g., declaring
        parameters, creating publishers). Override this method in subclasses as
        needed.
        """
        pass

    def on_shutdown(self):
        """
        Hook for subclasses to implement resource-specific shutdown logic.

        This method is called during node shutdown to allow resource-specific cleanup
        (e.g., destroying publishers, unsubscribing).
        Override this method in subclasses as needed.
        """
        pass


# Only for typing
class _RegisteredResource(Resource):

    __rosonic_name__: str           # pyright: ignore[reportIncompatibleVariableOverride]
    __rosonic_owner__: Resource     # pyright: ignore[reportIncompatibleVariableOverride]


# Only for typing
class _StartedResource(_RegisteredResource):

    __rosonic_node__: NodeBase


def _is_absolute_name(name: str) -> bool:
    return name.startswith('/') or name.startswith('~')

def _is_registered(resource: Resource) -> TypeGuard[_RegisteredResource]:
    return resource.__rosonic_owner__ is not None

def _is_started(resource: Resource) -> TypeGuard[_StartedResource]:
    return resource.__rosonic_node__ is not None

def _is_root(resource: Resource) -> bool:
    return resource.__rosonic_owner__ is resource

def _get_root(resource: Resource) -> Resource:
    assert _is_registered(resource), f"Resource '{resource}' is not registered"
    return (resource if _is_root(resource) else
            _get_root(resource))

class Node(Resource, NodeBase):
>>>>>>> f6498d4 (update rosonic)
    """
    Base class for all SVEA nodes.
    This class provides a simple interface for creating ROS 2 nodes with
    common functionality such as logging and lifecycle management.
    """

    @classmethod
    def main(cls, args=None):
        """
        Main function to run the SVEA node.
        """
        rclpy.init(args=args)

        node = cls()
<<<<<<< HEAD
<<<<<<< HEAD
=======
        name = node.get_name()
>>>>>>> 11eeed9 (Fundamental changes.)
=======
>>>>>>> 24b9acb (minor structural changes to rosonic)
        logger = node.get_logger()
        logger.debug("Initialization complete.")

        logger.info("Starting up...")
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        node.__rosonic_startup__(node)
=======
        node._startup()
>>>>>>> 11eeed9 (Fundamental changes.)
=======
        node._startup(node)
>>>>>>> 4b0286b (More work on simulator)
=======
        node.__rosonic_startup__(node)
>>>>>>> f6498d4 (update rosonic)
        logger.debug("Startup complete.")

        logger.info("Running...")
        try:
            node.run()
        except KeyboardInterrupt:
            pass
        finally:

            logger.info("Shutting down...")
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            node.__rosonic_shutdown__(node)
            logger.debug("Shutdown complete.")

            # # Perhaps causes more errors than help 
            # node.destroy_node()
            # rclpy.shutdown()

    def __init__(self, name: str | None = None, **kwds):
        name = name if name is not None else type(self).__name__
        Resource.__init__(self)
        NodeBase.__init__(self, name, **kwds)
        self.__rosonic_register__(self, name=self.get_fully_qualified_name())

=======
            node._shutdown()
=======
            node._shutdown(node)
>>>>>>> 4b0286b (More work on simulator)
=======
            node.__rosonic_shutdown__(node)
>>>>>>> f6498d4 (update rosonic)
            logger.debug("Shutdown complete.")

            # # Perhaps causes more errors than help 
            # node.destroy_node()
            # rclpy.shutdown()

    def __init__(self, name: str | None = None, **kwds):
        name = name if name is not None else type(self).__name__
        Resource.__init__(self)
        NodeBase.__init__(self, name, **kwds)
        self.__rosonic_register__(self, name=self.get_fully_qualified_name())

<<<<<<< HEAD
        ## Service initialization

        # TODO?

        ## Publisher initialization
        
        # TODO?

        ## Subscriber initialization

        # TODO?

        ## Timer initialization

        # TODO?

<<<<<<< HEAD
    def _startup(self):
        """
        Called when the node is started.
        This method is used to declare parameters and set up the node.
        """

        for member in dir(self):
            if isinstance(member, Member) and hasattr(member, 'on_startup'):
                member.on_startup(self)

        if hasattr(self, 'on_startup'):
            self.on_startup()

    def _shutdown(self):
        """
        Called when the node is shutting down.
        This method is used to clean up resources and stop the node.
        """
        if hasattr(self, 'on_shutdown'):
            self.on_shutdown()

        for member in dir(self):
            if isinstance(member, Member) and hasattr(member, 'on_shutdown'):
                member.on_shutdown(self)
        
>>>>>>> 11eeed9 (Fundamental changes.)
=======
>>>>>>> 4b0286b (More work on simulator)
=======
>>>>>>> 24b9acb (minor structural changes to rosonic)
    def run(self):
        """
        Main loop for the node.
        This method is called to start the node and run the main loop.
        """
<<<<<<< HEAD
<<<<<<< HEAD
        rclpy.spin(self)

class Field(Resource):
    """
    Structural resource container that does not reserve a name in the ROS graph.

    A Field is used to group child resources logically without introducing a
    named level in the graph namespace (unless explicitly specified). This
    allows you to organize your code hierarchically, while keeping the graph
    flat if desired.

    - If the Field has a `name`, it acts as a scoping element.
    - If not, its children are considered part of the nearest named ancestor.

    Example Usage:
    ```python
    class Interface(rx.Field):
        param1 = rx.Parameter('value')
        pub1 = rx.Publisher(MyMsg, 'topic')

    class MyNode1(rx.Node):
        iface = Interface(name='interface_ns')  # Children scoped under ~interface_ns

    class MyNode2(rx.Node):
        iface = Interface() # Children scoped directly under node
    ```

    Notes:
    - Fields can still act as logical parents for resource lookup and lifecycle hooks.
    - Naming behavior is **flattened** if no name/namespace is provided.
    - Fields cannot be directly used as graph resources (unless named).
    """

    def __set_name__(self, owner: type, name: str) -> None:
        """
        Called when the Field is assigned as a class attribute.
        Registers this Field as a child of the owner.

        Args:
            owner (type): The class to which this Field is being attached.
            name (str): The attribute name of this Field in the class.
        """
        assert issubclass(owner, Resource), f"Owner '{owner}' must be subclass of '{Resource}'"
        owner.__rosonic_preregistered__ += (self,)

    @property
    def node(self) -> NodeBase:
        """
        Provides access to the underlying ROS node.

        Returns:
            NodeBase: The ROS node associated with this Field.
        """
        return self.__rosonic_node__

class NamedField(Field):
    """
    Field that reserves a name in the ROS graph.

    A NamedField behaves like Field, but it will always assign itself a name,
    which makes it appear explicitly in the resource namespace.

    If a name is not provided during instantiation, it defaults to the attribute
    name assigned in the class definition.

    Example Usage:
    ```python
    class MyNode(rx.Node):
        iface = rx.NamedField()  # Will be named 'iface' in the graph
    ```

    Result:
    - The NamedField 'iface' appears as `/my_node/iface` in the graph.
    - Its children are scoped under it (e.g., `/my_node/iface/param1`).

    Notes:
    - NamedFields are used when you want explicit namespace levels for grouping.
    - Inheriting the class attribute name is intuitive especially for Parameters.
    """

    def __set_name__(self, owner: type, name: str) -> None:
        """
        Called when the NamedField is assigned as a class attribute.
        Assigns a name to the field if not already specified.

        Args:
            owner (type): The class to which this NamedField is being attached.
            name (str): The attribute name of this NamedField in the class.
        """
        assert issubclass(owner, Resource), f"Owner '{owner}' must be subclass of '{Resource}'"

        if self.__rosonic_name__ is None:
            self.__rosonic_name__ = name

        owner.__rosonic_preregistered__ += (self,)

class Parameter(NamedField):
    """
    Declarative Parameter resource.

    A `Parameter` declares a ROS parameter in the node's parameter server. It
    is defined declaratively as a class field and automatically handled during
    node startup. Parameters can be used as configuration values or even as
    dynamic topic names for other resources (e.g., Publishers).

    ### Example Usage:
    ```python
    class MyNode(rx.Node):
        threshold = rx.Parameter(0.5)
    ```

    The parameter `/my_node/threshold` will be declared when the node starts.

    ### Accessing Parameter Values:
    After startup, accessing the field (e.g., `self.threshold`) will return the
    current parameter value.

    ### Notes:
    - The parameter is not resolved (no value) until node startup is complete.
    - Until then, accessing the field will return the Parameter object itself.
    """

    def __init__(self, *args, name: str | None = None):
        """
        Initializes the Parameter resource.

        Args:
            *args: Default value and optional parameter descriptor arguments
            passed to `declare_parameter()`. 
            name (str | None): Optional name override. Defaults to attribute
            name if omitted.
        """
        super().__init__(name=name)
        self.args = args
        self.value = ... # Ellipsis used to detect when value hasn't been set yet
    
    def __get__(self, instance, owner):
        if instance is None:
            return self

<<<<<<< HEAD
        if not self._is_started():
=======
        if not _is_started(self):
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
            return self

        return self.value
    
    def on_startup(self):
        node = self.__rosonic_node__
        name = self.__rosonic_relname__
        
        if self.value is ...:
            node.declare_parameter(name, *self.args)
            self.value = node.get_parameter(name).value
    
class Publisher(NamedField):
    """
    Declarative ROS 2 Publisher resource.

    A `Publisher` declares a ROS publisher that is instantiated automatically
    during node startup. It can publish messages directly via method calls or
    through its callable interface.

    ### Example Usage:
    ```python



    class App(rx.Node):
    
        # Topic: /app/cmd_vel
        cmd_vel = rx.Publisher(Twist) 

        @rx.Timer(0.1)
        def loop(self):
            msg = Twist()
            msg.linear.x = 1.0
            self.cmd_vel.publish(msg)

    class MyNode(rx.Node):
        chat_pub = rx.Publisher(String, 'chat_topic')

        def on_startup(self):
            msg = String(data="Hello")
            self.chat_pub.publish(msg)
    ```

    ### Usage:
    - Use `self.chat_pub.publish(msg)` to publish a message.
    - You can also call `self.chat_pub(msg)` directly (callable shortcut).

    ### Notes:
    - Topic names can be dynamic if specified via a `Parameter`.
    - The publisher is created at startup and is unavailable before that.
    """

    def __init__(self, msg_type, topic=None, qos_profile=None):
        """
        Initializes the Publisher resource.

        Args:
            msg_type (type): The message type (e.g., `std_msgs.msg.String`).
            topic (str | Parameter): Topic name as string or Parameter
            reference.
            qos_profile: Optional QoSProfile for the publisher.
        """
        super().__init__(name=topic if topic is str else None)
        self.msg_type = msg_type
        self.topic = topic
=======
        rclpy.spin(self)

class Field(Resource):
    """
    Structural resource container that does not reserve a name in the ROS graph.

    A Field is used to group child resources logically without introducing a
    named level in the graph namespace (unless explicitly specified). This
    allows you to organize your code hierarchically, while keeping the graph
    flat if desired.

    - If the Field has a `name`, it acts as a scoping element.
    - If not, its children are considered part of the nearest named ancestor.

    Example Usage:
    ```python
    class Interface(rx.Field):
        param1 = rx.Parameter('value')
        pub1 = rx.Publisher(MyMsg, 'topic')

    class MyNode1(rx.Node):
        iface = Interface(name='interface_ns')  # Children scoped under ~interface_ns

    class MyNode2(rx.Node):
        iface = Interface() # Children scoped directly under node
    ```

    Notes:
    - Fields can still act as logical parents for resource lookup and lifecycle hooks.
    - Naming behavior is **flattened** if no name/namespace is provided.
    - Fields cannot be directly used as graph resources (unless named).
    """

    def __set_name__(self, owner: type, name: str) -> None:
        """
        Called when the Field is assigned as a class attribute.
        Registers this Field as a child of the owner.

        Args:
            owner (type): The class to which this Field is being attached.
            name (str): The attribute name of this Field in the class.
        """
        assert issubclass(owner, Resource), f"Owner '{owner}' must be subclass of '{Resource}'"
        owner.__rosonic_preregistered__ += (self,)

    @property
    def node(self) -> NodeBase:
        """
        Provides access to the underlying ROS node.

        Returns:
            NodeBase: The ROS node associated with this Field.
        """
        return self.__rosonic_node__

class NamedField(Field):
    """
    Field that reserves a name in the ROS graph.

    A NamedField behaves like Field, but it will always assign itself a name,
    which makes it appear explicitly in the resource namespace.

    If a name is not provided during instantiation, it defaults to the attribute
    name assigned in the class definition.

    Example Usage:
    ```python
    class MyNode(rx.Node):
        iface = rx.NamedField()  # Will be named 'iface' in the graph
    ```

    Result:
    - The NamedField 'iface' appears as `/my_node/iface` in the graph.
    - Its children are scoped under it (e.g., `/my_node/iface/param1`).

    Notes:
    - NamedFields are used when you want explicit namespace levels for grouping.
    - Inheriting the class attribute name is intuitive especially for Parameters.
    """

    def __set_name__(self, owner: type, name: str) -> None:
        """
        Called when the NamedField is assigned as a class attribute.
        Assigns a name to the field if not already specified.

        Args:
            owner (type): The class to which this NamedField is being attached.
            name (str): The attribute name of this NamedField in the class.
        """
        assert issubclass(owner, Resource), f"Owner '{owner}' must be subclass of '{Resource}'"

        if self.__rosonic_name__ is None:
            self.__rosonic_name__ = name

        owner.__rosonic_preregistered__ += (self,)

class Parameter(NamedField):
    """
    Declarative Parameter resource.

    A `Parameter` declares a ROS parameter in the node's parameter server. It
    is defined declaratively as a class field and automatically handled during
    node startup. Parameters can be used as configuration values or even as
    dynamic topic names for other resources (e.g., Publishers).

    ### Example Usage:
    ```python
    class MyNode(rx.Node):
        threshold = rx.Parameter(0.5)
    ```

    The parameter `/my_node/threshold` will be declared when the node starts.

    ### Accessing Parameter Values:
    After startup, accessing the field (e.g., `self.threshold`) will return the
    current parameter value.

    ### Notes:
    - The parameter is not resolved (no value) until node startup is complete.
    - Until then, accessing the field will return the Parameter object itself.
    """

    def __init__(self, *args, name: str | None = None):
        """
        Initializes the Parameter resource.

        Args:
            *args: Default value and optional parameter descriptor arguments
            passed to `declare_parameter()`. 
            name (str | None): Optional name override. Defaults to attribute
            name if omitted.
        """
        super().__init__(name=name)
        self.args = args
        self.value = ... # Ellipsis used to detect when value hasn't been set yet
    
    def __get__(self, instance, owner):
        if instance is None:
            return self

        if not _is_started(self):
            return self

        return self.value
    
    def on_startup(self):
        node = self.__rosonic_node__
        name = self.__rosonic_relname__
        
        if self.value is ...:
            node.declare_parameter(name, *self.args)
            self.value = node.get_parameter(name).value
    
class Publisher(NamedField):
    """
    Declarative ROS 2 Publisher resource.

    A `Publisher` declares a ROS publisher that is instantiated automatically
    during node startup. It can publish messages directly via method calls or
    through its callable interface.

    ### Example Usage:
    ```python
    class MyNode(rx.Node):
        chat_pub = rx.Publisher(String, 'chat_topic')

        def on_startup(self):
            msg = String(data="Hello")
            self.chat_pub.publish(msg)
    ```

    ### Usage:
    - Use `self.chat_pub.publish(msg)` to publish a message.
    - You can also call `self.chat_pub(msg)` directly (callable shortcut).

    ### Notes:
    - Topic names can be dynamic if specified via a `Parameter`.
    - The publisher is created at startup and is unavailable before that.
    """

    def __init__(self, msg_type, topic=None, qos_profile=None):
        """
        Initializes the Publisher resource.

        Args:
            msg_type (type): The message type (e.g., `std_msgs.msg.String`).
            topic (str | Parameter): Topic name as string or Parameter
            reference.
            qos_profile: Optional QoSProfile for the publisher.
        """
        super().__init__(name=topic if topic is str else None)
        self.msg_type = msg_type
<<<<<<< HEAD
>>>>>>> 24b9acb (minor structural changes to rosonic)
=======
        self.topic = topic
>>>>>>> f6498d4 (update rosonic)
        self.qos_profile = qos_profile
        self.publisher = None

    def __call__(self, *args, **kwds):
<<<<<<< HEAD
<<<<<<< HEAD
=======
        """
        Publish a message to the topic.
        Raise an exception if the publisher is not started.
        """
>>>>>>> 24b9acb (minor structural changes to rosonic)
=======
>>>>>>> f6498d4 (update rosonic)
        return self.publish(*args, **kwds)

    def publish(self, msg):
        """
        Publish a message to the topic.
        Raise an exception if the publisher is not started.
        """
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        assert self._is_started(), f"Publisher for topic '{self.topic}' is not started yet."
=======
        assert _is_started(self), f"Publisher for topic '{self.topic}' is not started yet."
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
=======
=======
>>>>>>> e1d0aaa (update rosonic)
        assert _is_started(self), f"Publisher for topic '{self.topic}' is not started yet."
=======
        if self.publisher is None:
            raise RuntimeError(f"Publisher for topic '{self.topic}' is not started yet.")
>>>>>>> 24b9acb (minor structural changes to rosonic)
<<<<<<< HEAD
>>>>>>> 6525281 (minor structural changes to rosonic)
=======
=======
        assert _is_started(self), f"Publisher for topic '{self.topic}' is not started yet."
>>>>>>> f6498d4 (update rosonic)
>>>>>>> e1d0aaa (update rosonic)
        self.publisher.publish(msg)

    def on_startup(self):
        """
        Called when the node is started.
        This method is used to create the publisher in the node.
        """
        node = self.__rosonic_node__
        msg_type = self.msg_type
<<<<<<< HEAD
<<<<<<< HEAD
        topic = (self.topic if self.topic is not None else 
                 self.__rosonic_fullname__)
        qos_profile = self.qos_profile or rclpy.qos.qos_profile_default

        if isinstance(topic, Parameter):
<<<<<<< HEAD
            assert topic._is_started(), f"Resource '{self}' depend on '{topic}' which has not started yet"
=======
            assert _is_started(topic), f"Resource '{self}' depend on '{topic}' which has not started yet"
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
            topic = topic.value
=======
        topic = self.topic
        qos_profile = self.qos_profile

        if isinstance(topic, Parameter):
<<<<<<< HEAD
            topic = self.__rosonic_node__.__rosonic_parameters__[topic._name].value
>>>>>>> 24b9acb (minor structural changes to rosonic)
=======
            topic = self.__rosonic_node__.__rosonic_parameters__[topic.name].value
>>>>>>> 2b88bc9 (fix some bugs in rosnoic.Subscriber, fix issue about odometry from sim_svea being slow)
=======
        topic = (self.topic if self.topic is not None else 
                 self.__rosonic_fullname__)
        qos_profile = self.qos_profile or rclpy.qos.qos_profile_default

        if isinstance(topic, Parameter):
            assert _is_started(topic), f"Resource '{self}' depend on '{topic}' which has not started yet"
            topic = topic.value
>>>>>>> f6498d4 (update rosonic)

        if not isinstance(msg_type, type):
            raise RuntimeError(f"Message type must be a class, not {type(msg_type)}")
        if not isinstance(topic, str):
            raise RuntimeError(f"Topic name must be a string, not {type(topic)}")
        if not isinstance(qos_profile, rclpy.qos.QoSProfile):
            raise RuntimeError(f"QoS profile must be a QoSProfile, not {type(qos_profile)}")

        self.publisher = node.create_publisher(msg_type, topic, qos_profile)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
class Subscriber(NamedField):
    """
    Declarative ROS 2 Subscriber resource.

    A `Subscriber` declares a ROS subscription, associating a callback to a
    topic. The subscription is created automatically during node startup. The
    resource is defined declaratively and the callback is attached via
    decorator syntax.

    ### Example Usage:
    ```python
    class MyNode1(rx.Node):

        listener = rx.Subscriber(String, 'chatter')

        @listener
        def on_message(self, msg):
            self.get_logger().info(f"Received: {msg.data}")

    class MyNode2(rx.Node):

        @rx.Subscriber(String, 'chatter')
        def on_message(self, msg):
            self.get_logger().info(f"Received: {msg.data}")
    ```

    ### Usage:
    - Decorate a method using `@subscriber_field` to assign the callback.
    - The callback receives `(self, msg)` where `self` is the owning resource
      (Node).

    ### Notes:
    - Topic names can be dynamic if specified via a `Parameter`.
    - The subscription is created at startup and unavailable before that.
    - If no callback is assigned via decorator, startup will raise an error.
    """

    def __init__(self, msg_type, topic=None, qos_profile=None):
        """
        Initializes the Subscriber resource.

        Args:
            msg_type (type): The message type (e.g., `std_msgs.msg.String`).
            name (str | Parameter): Topic name as string or Parameter
            reference.
            qos_profile: Optional QoSProfile for the subscriber.
        """

        super().__init__(name=topic if topic is str else None)
        self.subscriber = None
        self.msg_type = msg_type
        self.topic = topic
=======

=======
>>>>>>> 217dc92 (05/12/2025 meeting update)
class Subscriber(Member):
=======
class Subscriber(NamedField):
>>>>>>> f6498d4 (update rosonic)
    """
    Declarative ROS 2 Subscriber resource.

    A `Subscriber` declares a ROS subscription, associating a callback to a
    topic. The subscription is created automatically during node startup. The
    resource is defined declaratively and the callback is attached via
    decorator syntax.

    ### Example Usage:
    ```python
    class MyNode1(rx.Node):

        listener = rx.Subscriber(String, 'chatter')

        @listener
        def on_message(self, msg):
            self.get_logger().info(f"Received: {msg.data}")

    class MyNode2(rx.Node):

        @rx.Subscriber(String, 'chatter')
        def on_message(self, msg):
            self.get_logger().info(f"Received: {msg.data}")
    ```

    ### Usage:
    - Decorate a method using `@subscriber_field` to assign the callback.
    - The callback receives `(self, msg)` where `self` is the owning resource
      (Node).

    ### Notes:
    - Topic names can be dynamic if specified via a `Parameter`.
    - The subscription is created at startup and unavailable before that.
    - If no callback is assigned via decorator, startup will raise an error.
    """

    def __init__(self, msg_type, topic=None, qos_profile=None):
        """
        Initializes the Subscriber resource.

        Args:
            msg_type (type): The message type (e.g., `std_msgs.msg.String`).
            name (str | Parameter): Topic name as string or Parameter
            reference.
            qos_profile: Optional QoSProfile for the subscriber.
        """

        super().__init__(name=topic if topic is str else None)
        self.subscriber = None
        self.msg_type = msg_type
<<<<<<< HEAD
>>>>>>> 24b9acb (minor structural changes to rosonic)
=======
        self.topic = topic
>>>>>>> f6498d4 (update rosonic)
        self.qos_profile = qos_profile
        self.subscriber = None
        self.callback = None

    def __call__(self, callback):
        """
        Use this subscriber as a decorator to set the callback function.
        """
        self.callback = callback
        return self

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> f6498d4 (update rosonic)
    def __getattr__(self, name: str):
        if self.subscriber is None or not hasattr(self.subscriber, name):
            raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")
        return getattr(self.subscriber, name)

<<<<<<< HEAD
=======
>>>>>>> 24b9acb (minor structural changes to rosonic)
=======
>>>>>>> f6498d4 (update rosonic)
    def on_startup(self):
        """
        Called when the node is started.
        This method is used to create the subscriber in the node.
        """

        node = self.__rosonic_node__
        msg_type = self.msg_type
<<<<<<< HEAD
<<<<<<< HEAD
        topic = (self.topic if self.topic is not None else 
                 self.__rosonic_fullname__)
        qos_profile = self.qos_profile or rclpy.qos.qos_profile_default

        if isinstance(topic, Parameter):
<<<<<<< HEAD
            assert topic._is_started(), f"Resource '{self}' depend on '{topic}' which has not started yet"
=======
            assert _is_started(topic), f"Resource '{self}' depend on '{topic}' which has not started yet"
>>>>>>> ecc9d3f (Migration to ROS 2 (#55))
            topic = topic.value
=======
        topic = self.topic
        qos_profile = self.qos_profile

        if isinstance(topic, Parameter):
<<<<<<< HEAD
            topic = self.__rosonic_node__.__rosonic_parameters__[topic._name].value
>>>>>>> 24b9acb (minor structural changes to rosonic)
=======
            topic = self.__rosonic_node__.__rosonic_parameters__[topic.name].value
>>>>>>> 2b88bc9 (fix some bugs in rosnoic.Subscriber, fix issue about odometry from sim_svea being slow)
=======
        topic = (self.topic if self.topic is not None else 
                 self.__rosonic_fullname__)
        qos_profile = self.qos_profile or rclpy.qos.qos_profile_default

        if isinstance(topic, Parameter):
            assert _is_started(topic), f"Resource '{self}' depend on '{topic}' which has not started yet"
            topic = topic.value
>>>>>>> f6498d4 (update rosonic)

        if not isinstance(msg_type, type):
            raise RuntimeError(f"Message type must be a class, not {type(msg_type)}")
        if not isinstance(topic, str):
            raise RuntimeError(f"Topic name must be a string, not {type(topic)}")
        if not isinstance(qos_profile, rclpy.qos.QoSProfile):
            raise RuntimeError(f"QoS profile must be a QoSProfile, not {type(qos_profile)}")

        if self.callback is None:
            raise RuntimeError(f"Callback for subscriber '{self.topic}' is not set.")

<<<<<<< HEAD
<<<<<<< HEAD
        user_callback = self.callback
        owner = self.__rosonic_owner__
<<<<<<< HEAD

        def wrapped_callback(msg):
            """
            Wrapper for the user-defined callback to include the parent context.
            """
            return user_callback(owner, msg)

        self.subscriber = node.create_subscription(msg_type, topic, wrapped_callback, qos_profile)

class Timer(NamedField):
    """
    Declarative ROS 2 Timer resource.

    A `Timer` defines a periodic callback that is instantiated automatically
    during node startup. It is declared declaratively and the callback is
    attached using a decorator syntax.

    ### Example Usage:
    ```python
    class MyNode(rx.Node):

        @rx.Timer(1.0)  # Executes every 1 second
        def my_loop(self):
            self.get_logger().info("Tick")
    ```

    ### Usage:
    - Decorate a method using `@Timer(...)` to assign the callback.
    - The callback receives `(self)` where `self` is the owning resource (Node or NamedField).

    ### Accessing Timer Methods:
    After startup, you can access timer methods like `.reset()`, `.cancel()`, etc., through the Timer field.

    ### Example with autostart:
    ```python
    @rx.Timer(1.0, autostart=False)
    def delayed_loop(self):
        ...
    
    def on_startup(self):
        self.delayed_loop.reset()  # Manually start the timer
    ```

    ### Notes:
    - The Timer is not created until node startup.
    - If `autostart=False` is specified, you must manually call `.reset()` to start it.
    - Timer methods are proxied after startup (e.g., `self.timer_field.reset()`).
    """

    def __init__(self, period: float,
                 name: str | None = None, **kwds):
        """
        Initializes the Timer resource.

        Args:
            period (float): Timer period in seconds.
            name (str | None): Optional resource name. Defaults to attribute
            name if omitted.
            **kwds: Additional keyword arguments passed to `create_timer()`,
            such as `autostart=False`.
        """

        super().__init__(name=name)
        self.tmr = None
        self.period = period
        self.callback = None
        self.kwds = kwds

    def __call__(self, callback):
        """X"""
        self.callback = callback
        return self

    def __getattr__(self, name: str):
        if self.tmr is None or not hasattr(self.tmr, name):
            raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")
        return getattr(self.tmr, name)

    def on_startup(self):
        node = self.__rosonic_node__
        owner = self.__rosonic_owner__
        period = self.period
        if isinstance(period, Parameter):
            assert _is_started(period), f"Resource '{self}' depend on '{period}' which has not started yet"
            period = period.value
        wrapped_callback = lambda: self.callback(owner)
        self.tmr = node.create_timer(period, wrapped_callback, **self.kwds)

=======
        rclpy.spin(self)
>>>>>>> 11eeed9 (Fundamental changes.)
=======
        callback = self.callback
=======
        user_callback = self.callback
>>>>>>> 2b88bc9 (fix some bugs in rosnoic.Subscriber, fix issue about odometry from sim_svea being slow)
        parent = self.__rosonic_parent__
=======
>>>>>>> f6498d4 (update rosonic)

        def wrapped_callback(msg):
            """
            Wrapper for the user-defined callback to include the parent context.
            """
            return user_callback(owner, msg)

        self.subscriber = node.create_subscription(msg_type, topic, wrapped_callback, qos_profile)

class Timer(NamedField):
    """
    Declarative ROS 2 Timer resource.

    A `Timer` defines a periodic callback that is instantiated automatically
    during node startup. It is declared declaratively and the callback is
    attached using a decorator syntax.

    ### Example Usage:
    ```python
    class MyNode(rx.Node):

        @rx.Timer(1.0)  # Executes every 1 second
        def my_loop(self):
            self.get_logger().info("Tick")
    ```

    ### Usage:
    - Decorate a method using `@Timer(...)` to assign the callback.
    - The callback receives `(self)` where `self` is the owning resource (Node or NamedField).

    ### Accessing Timer Methods:
    After startup, you can access timer methods like `.reset()`, `.cancel()`, etc., through the Timer field.

    ### Example with autostart:
    ```python
    @rx.Timer(1.0, autostart=False)
    def delayed_loop(self):
        ...
    
    def on_startup(self):
        self.delayed_loop.reset()  # Manually start the timer
    ```

    ### Notes:
    - The Timer is not created until node startup.
    - If `autostart=False` is specified, you must manually call `.reset()` to start it.
    - Timer methods are proxied after startup (e.g., `self.timer_field.reset()`).
    """

    def __init__(self, period: float,
                 name: str | None = None, **kwds):
        """
        Initializes the Timer resource.

        Args:
            period (float): Timer period in seconds.
            name (str | None): Optional resource name. Defaults to attribute
            name if omitted.
            **kwds: Additional keyword arguments passed to `create_timer()`,
            such as `autostart=False`.
        """

        super().__init__(name=name)
        self.tmr = None
        self.period = period
        self.callback = None
        self.kwds = kwds

    def __call__(self, callback):
        """X"""
        self.callback = callback
        return self

    def __getattr__(self, name: str):
        if self.tmr is None or not hasattr(self.tmr, name):
            raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")
        return getattr(self.tmr, name)

    def on_startup(self):
        node = self.__rosonic_node__
        owner = self.__rosonic_owner__
        period = self.period
        if isinstance(period, Parameter):
            assert _is_started(period), f"Resource '{self}' depend on '{period}' which has not started yet"
            period = period.value
        wrapped_callback = lambda: self.callback(owner)
        self.tmr = node.create_timer(period, wrapped_callback, **self.kwds)

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> 24b9acb (minor structural changes to rosonic)
=======
        self.subscriber = node.create_subscription(msg_type, topic, wrapped_callback, qos_profile)
>>>>>>> 2b88bc9 (fix some bugs in rosnoic.Subscriber, fix issue about odometry from sim_svea being slow)
=======
>>>>>>> f6498d4 (update rosonic)
