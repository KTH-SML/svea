
from threading import Thread
import rospy
from std_msgs.msg import Float32
from math import radians


class SpeedGuard():
    """
    Used to handle speed constraints.
    """
    MSG_RATE = 1 # Maximum log message rate in seconds
    MIN_ANGLE = radians(10)

    def __init__(self) -> None:
        self._speed_multiplier = 1.0
        self.object_detected = False
        self.drive_slow = False
        self.stand_still = False
        self.last_angle = 0.0
        

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
        rospy.loginfo("Starting Speed Guard Node")
        self._start_listen()
        self._start_publish()

        rospy.loginfo("Speed Guard successfully initialized")
        rospy.spin()

    def _start_listen(self):
        rospy.Subscriber('speed_multiplier', Float32, self._speed_multiplier_callback, 
                                                                    tcp_nodelay=True)

    def _start_publish(self):
        self._steering_angle_pub = rospy.Publisher(
            '/steering_angle', Float32, queue_size=1, tcp_nodelay=True)

    def _speed_multiplier_callback(self, speed_msg):
        self._speed_multiplier = speed_msg.data


    def check_obstacles(self, verbose=False) -> bool:

        if verbose: # Log all objects detected
            if self._speed_multiplier < 1: 
                if not self.object_detected:
                    rospy.logwarn_throttle(self.MSG_RATE, "Object detected")
                    self.object_detected = True
                if self.emergency and not self.stand_still:
                    rospy.logwarn("Obstacle detected, stopping vehicle")                    
                    self.stand_still = True
                    self.drive_slow = False
                elif not self.emergency and not self.drive_slow:
                    rospy.loginfo_throttle(self.MSG_RATE, "Driving slowly")   
                    self.stand_still = False
                    self.drive_slow = True
            elif self.object_detected:
                rospy.loginfo_throttle(self.MSG_RATE, "Obstacle cleared")
                self.object_detected = False
                self.stand_still = False
                self.drive_slow = False
                
        else: # Log only when stopping for an obstacle
            if self.multiplier == 0:
                if not self.stand_still:
                    rospy.logwarn("Obstacle detected, stopping vehicle")
                    self.stand_still = True
            elif self.stand_still:
                rospy.loginfo("Obstacle cleared")
                self.stand_still = False

        return self.stand_still

    def publish_steering_angle(self, angle):
        if abs(angle) >= self.MIN_ANGLE:
            self.last_angle = angle
            self._steering_angle_pub.publish(angle)
        elif abs(self.last_angle) > self.MIN_ANGLE:
            self._steering_angle_pub.publish(0.0)
            self.last_angle = 0.0

    @property
    def emergency(self):
        return self._speed_multiplier == 0.0

    @property
    def multiplier(self):
        return self._speed_multiplier