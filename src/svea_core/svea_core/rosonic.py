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

    __rosonic_node__ = None
    __rosonic_name__ = None
    __rosonic_active__ = False
    __rosonic_parent__ = None
    __rosonic_members__ = ()

    def __set_name__(self, owner, name):
        
        self.__rosonic_name__ = name

        if issubclass(owner, Member):
            owner.__rosonic_members__ += ((name, self),)

    def _rosonic_members(self):
        """
        Return a dictionary of all rosonic members of the node.
        This method is used to get all the members of the node that are
        rosonic members. It returns a dictionary with the member names as keys
        and the member objects as values.
        """
        return dict(self.__rosonic_members__)

    def _startup(self, node):
        """
        Called when the node is started.
        This method is used to declare parameters and set up the node.
        """
        
        for member in self._rosonic_members().values():
            member.__rosonic_parent__ = self
            member._startup(node)

        self.__rosonic_node__ = node
        self.__rosonic_active__ = True

        if hasattr(self, 'on_startup'):
            self.on_startup()
        
    def _shutdown(self, node):
        """
        Called when the node is shutting down.
        This method is used to clean up resources and stop the node.
        """

        if hasattr(self, 'on_shutdown'):
            self.on_shutdown()

        self.__rosonic_active__ = False

        for member in self._rosonic_members().values():
            member._shutdown(node)

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


class Node(Member, rclpy.node.Node):
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
        name = node.get_name()
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

        ## Parameter initialization

        self.__rosonic_parameters__ = {}

        ## Service initialization

        # TODO?

        ## Publisher initialization
        
        # TODO?

        ## Subscriber initialization

        # TODO?

        ## Timer initialization

        # TODO?

    def run(self):
        """
        Main loop for the node.
        This method is called to start the node and run the main loop.
        """
        rclpy.spin(self)