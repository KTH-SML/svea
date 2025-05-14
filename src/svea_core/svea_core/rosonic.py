"""
This started as a fun project in ROS 1 to make nodes easier to write. With 
ROS 2, I thought rosonoic wouldn't be necessary. However, unsurprisingly, ROS
took one step forward and two steps back. So, here we are again.

Kaj Munhoz Arfvidsson
"""

from typing import Optional

import rclpy

__all__ = [
    'Node',
    'Member',
    'Parameter',
    'Publisher',
    'Subscriber',
]

class Member:
    """
    Base class for all rosonic members of a ROS 2 node.

    A rosonic member is a class that has specific hooks for, e.g., initialization
    and shutdown. This class is used to define the interface for all rosonic
    members, such as parameters, services, publishers, subscribers, and timers.
    This class is not meant to be used directly, but rather as a base class for
    other classes that implement specific functionality.
    """

    __rosonic_node__: Optional['Node']      = None
    __rosonic_name__: Optional[str]         = None
    __rosonic_active__: bool                = False
    __rosonic_parent__: Optional['Member']  = None
    __rosonic_members__: tuple              = ()    # ((name, member), ...)

    def __set_name__(self, owner: type, name: str) -> None:
        
        self.__rosonic_name__ = name

        if issubclass(owner, Member):
            owner.__rosonic_members__ += ((name, self),)

    def _rosonic_members(self) -> dict[str, 'Member']:
        """
        Return a dictionary of all rosonic members of the node.
        This method is used to get all the members of the node that are
        rosonic members. It returns a dictionary with the member names as keys
        and the member objects as values.
        """
        return dict(self.__rosonic_members__)

    def _startup(self, node: 'Node') -> None:
        """
        Called when the node is started.
        This method is used to declare parameters and set up the node.
        """
        
        for member in self._rosonic_members().values():
            member.__rosonic_parent__ = self
            member._startup(node)

        self.__rosonic_node__ = node
        self.__rosonic_active__ = True

        self.on_startup()
        
    def _shutdown(self, node: 'Node') -> None:
        """
        Called when the node is shutting down.
        This method is used to clean up resources and stop the node.
        """

        self.on_shutdown()

        self.__rosonic_active__ = False

        for member in self._rosonic_members().values():
            member._shutdown(node)

class Node(Member, rclpy.node.Node):
    """
    Base class for all SVEA nodes.
    This class provides a simple interface for creating ROS 2 nodes with
    common functionality such as logging and lifecycle management.
    """

    __rosonic_parameters__: dict[str, rclpy.Parameter]

    @classmethod
    def main(cls, args=None):
        """
        Main function to run the SVEA node.
        """
        rclpy.init(args=args)

        node = cls()
        logger = node.get_logger()
        logger.debug("Initialization complete.")

        logger.info("Starting up...")
        node._startup(node)
        logger.debug("Startup complete.")

        logger.info("Running...")
        try:
            node.run()
        except KeyboardInterrupt:
            pass
        finally:

            logger.info("Shutting down...")
            node._shutdown(node)
            logger.debug("Shutdown complete.")

            # # Perhaps causes more errors than help 
            # node.destroy_node()
            # rclpy.shutdown()

    def __init__(self, name: Optional[str] = None):
        
        name = name if name is not None else type(self).__name__
        super().__init__(name)

        self.__rosonic_parameters__ = {}

    def run(self):
        """
        Main loop for the node.
        This method is called to start the node and run the main loop.
        """
        rclpy.spin(self)

class Resource(Member):
    """
    Base class for members that act as resources for a node.
    This class provides convenient access to fields and methods relevant to resources.

    TODO: 
    - Add namespace option.
    """

    @property
    def node(self) -> Optional[Node]:
        """
        Get the node associated with this resource.
        """
        return self.__rosonic_node__

    @property
    def active(self) -> bool:
        """
        Check if the resource is currently active.
        """
        return self.__rosonic_active__

    @property
    def name(self) -> Optional[str]:
        """
        Get the name of the resource.
        """
        return self.__rosonic_name__

class Parameter(Resource):
    """
    Class to represent a parameter in a ROS 2 node.
    This class is used to define parameters for the node and provides
    functionality to set and get parameter values.
    """

    def __init__(self, *args, name=None, **kwds):
        """
        Initialize the parameter with a name and an optional default value.
        """
        self.param_name = name
        self.param_args = args
        self.param_kwds = kwds

    def __get__(self, instance, owner):
        """
        Get the value of the parameter.
        """
        if instance is None:
            return self

        if not self.__rosonic_active__:
            return self

        parameters = self.node.__rosonic_parameters__
        
        return parameters[self.param_name].value
    
    def on_startup(self):
        """
        Called when the node is started.
        This method is used to declare the parameter in the node.
        """
        if self.param_name is None:
            self.param_name = self.__rosonic_name__
        
        parameters = self.node.__rosonic_parameters__

        if self.param_name not in parameters:
            self.node.declare_parameter(self.param_name, *self.param_args, **self.param_kwds)
            parameters[self.param_name] = self.node.get_parameter(self.param_name)
    
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
        self.topic = topic
        self.msg_type = msg_type
        self.qos_profile = qos_profile
        self.publisher = None

    def __call__(self, *args, **kwds):
        """
        Publish a message to the topic.
        Raise an exception if the publisher is not started.
        """
        return self.publish(*args, **kwds)

    def publish(self, msg):
        """
        Publish a message to the topic.
        Raise an exception if the publisher is not started.
        """
        if self.publisher is None:
            raise RuntimeError(f"Publisher for topic '{self.topic}' is not started yet.")
        self.publisher.publish(msg)

    def on_startup(self):
        """
        Called when the node is started.
        This method is used to create the publisher in the node.
        """
        node = self.__rosonic_node__
        msg_type = self.msg_type
        topic = self.topic
        qos_profile = self.qos_profile

        if isinstance(topic, Parameter):
            topic = self.__rosonic_node__.__rosonic_parameters__[topic.name].value

        if not isinstance(msg_type, type):
            raise RuntimeError(f"Message type must be a class, not {type(msg_type)}")
        if not isinstance(topic, str):
            raise RuntimeError(f"Topic name must be a string, not {type(topic)}")
        if not isinstance(qos_profile, rclpy.qos.QoSProfile):
            raise RuntimeError(f"QoS profile must be a QoSProfile, not {type(qos_profile)}")

        self.publisher = node.create_publisher(msg_type, topic, qos_profile)

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
        self.topic = topic
        self.msg_type = msg_type
        self.qos_profile = qos_profile
        self.subscriber = None
        self.callback = None

    def __call__(self, callback):
        """
        Use this subscriber as a decorator to set the callback function.
        """
        self.callback = callback
        return self

    def on_startup(self):
        """
        Called when the node is started.
        This method is used to create the subscriber in the node.
        """

        node = self.__rosonic_node__
        msg_type = self.msg_type
        topic = self.topic
        qos_profile = self.qos_profile

        if isinstance(topic, Parameter):
            topic = self.__rosonic_node__.__rosonic_parameters__[topic.name].value

        if not isinstance(msg_type, type):
            raise RuntimeError(f"Message type must be a class, not {type(msg_type)}")
        if not isinstance(topic, str):
            raise RuntimeError(f"Topic name must be a string, not {type(topic)}")
        if not isinstance(qos_profile, rclpy.qos.QoSProfile):
            raise RuntimeError(f"QoS profile must be a QoSProfile, not {type(qos_profile)}")

        if self.callback is None:
            raise RuntimeError(f"Callback for subscriber '{self.topic}' is not set.")

        user_callback = self.callback
        parent = self.__rosonic_parent__

        def wrapped_callback(msg):
            """
            Wrapper for the user-defined callback to include the parent context.
            """
            return user_callback(parent, msg)

        self.subscriber = node.create_subscription(msg_type, topic, wrapped_callback, qos_profile)