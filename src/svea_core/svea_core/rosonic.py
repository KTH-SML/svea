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

            node.destroy_node()
            rclpy.shutdown()

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