from threading import Thread
import math
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistWithCovarianceStamped


class LidarInterface:
    """
    Basic interface for handling a Lidar. Collects and stores the most recent
    scan.

    TODO: Update similar to LocalizationInterface.
    """

    def __init__(self, Node:Node):
        self.node = Node
        self.scan = []
        # list of functions to call whenever a new scan comes in
        self.callbacks = []

    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving and sending data

        :return: itself
        :rtype: Lidar
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        self.node.get_logger().info("Starting Lidar Interface Node: \n" + str(self))
        self._collect_srvs()
        self._start_listen()
        rclpy.spin(self.node)

    def _collect_srvs(self):
        pass

    def _start_listen(self):
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        self.node.create_subscription(LaserScan, 'scan', self._read_scan, qos_profile)
        # rospy.Subscriber('scan', LaserScan, self._read_scan,
        #                  tcp_nodelay=True)
        self.node.get_logger().info("Lidar Interface successfully initialized")
        # rclpy.loginfo("Lidar Interface successfully initialized")

    def _read_scan(self, scan_msg):
        self.scan = scan_msg.ranges

        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.angle_increment = scan_msg.angle_increment

        self.time_increment = scan_msg.time_increment

        self.last_scan_time = scan_msg.scan_time

        for cb in self.callbacks:
            cb(self.scan, self.angle_min, self.angle_increment)

    def add_callback(self, cb):
        """Add state callback. Every function passed into this method
        will be called whenever new scan information comes in from the
        Lidar driver.

        :param cb: A callback function intended for responding to the
                   reception of a new scan, function must accept list
                   of scans, min angle, and angle increment as arguments
        :type cb: function
        """
        self.callbacks.append(cb)

    def remove_callback(self, cb):
        """Remove callback so it will no longer be called when state
        information is received

        :param cb: A callback function that should be no longer used
                   in response to the reception of state info
        :type cb: function
        """
        while cb in self.callbacks:
            self.callbacks.pop(self.callbacks.index(cb))
