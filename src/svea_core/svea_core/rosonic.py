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
    pass

class Parameter(Member):
    """
    Class to represent a parameter in a ROS 2 node.
    This class is used to define parameters for the node and provides
    functionality to set and get parameter values.

    Usage:
        ```python
        import rosonic as rx

        class MyNode(rx.Node):
            meaning = rx.Parameter(42) # Name is 'meaning' and default value is 42

            @rx.Parameter('Hello!', name='printer')
            def printer_callback(self, value):
                self.get_logger().info(f"Parameter 'printer' changed to {value}")
        ```
    """

    def __init__(self, *args, name=None, **kwds):
        """
        Initialize the parameter with a name and an optional default value.
        """
        self._name = name
        self._args = args
        self._kwds = kwds

        self._callback = None
        self._parameter = None

    def __set_name__(self, owner, name):
        """
        Set the name of the parameter when it is assigned to a class.
        """
        if self._name is None:
            self._name = name

    def __call__(self, func):
        """
        Decorator to mark a method as a callback for the parameter.
        This method will be called when the parameter is set or changed.
        """
        self._callback = func
        return self

    def __get__(self, instance, owner):
        """
        Get the value of the parameter.
        """
        if instance is None:
            return self

        if self._callback is not None:
            return self
        
        return instance.get_parameter(self._name).value
    
    def on_startup(self, node):
        """
        Called when the node is started.
        This method is used to declare the parameter in the node.
        """
        if self._parameter is None:
            self._parameter = node.declare_parameter(self._name, *self._args, **self._kwds)

class Node(rclpy.node.Node):
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
        node._startup()
        logger.debug("Startup complete.")

        logger.info("Running...")
        try:
            node.run()
        except KeyboardInterrupt:
            pass
        finally:

            logger.info("Shutting down...")
            node._shutdown()
            logger.debug("Shutdown complete.")

            node.destroy_node()
            rclpy.shutdown()

    def __init__(self, name: Optional[str] = None):
        
        name = name if name is not None else type(self).__name__
        super().__init__(name)

        for member in dir(self):
            if isinstance(member, Member):
                pass

        ## Parameter initialization

        callbacks = {
            member._name: member._callback
            for member in dir(self) 
            if isinstance(member, Parameter)
        }

        def _parameter_callback(params):
            """
            Callback function to handle parameter changes.
            This function is called when a parameter is set or changed.
            """
            for param in params:
                if cb := callbacks.get(param.name, None):
                    cb(self, param.value)
                else:
                    self.get_logger().warn(f"No registered callback for {param.name}.")

        if len(callbacks) > 0:
            self.add_on_set_parameters_callback(_parameter_callback)
            self.get_logger().debug("* Added parameter callback.")
            
        ## Service initialization

        # TODO?

        ## Publisher initialization
        
        # TODO?

        ## Subscriber initialization

        # TODO?

        ## Timer initialization

        # TODO?

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
        
    def run(self):
        """
        Main loop for the node.
        This method is called to start the node and run the main loop.
        """
        rclpy.spin(self)