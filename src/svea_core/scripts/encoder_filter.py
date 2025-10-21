#! /usr/bin/env python3

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Bool, Int8

from svea_core import rosonic as rx

qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


qos_subber = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # BEST_EFFORT
    history=QoSHistoryPolicy.KEEP_LAST,         # Keep the last N messages
    durability=QoSDurabilityPolicy.VOLATILE,    # Volatile
    depth=10,                                   # Size of the queue
)


class encoder_filter(rx.Node):
    """
    Correcting Encoder Diraction
    """

    rm_throttle = 1 
    ctrl_throttle = 1
    override = None
    encoder_time = None

    ## Publishers ##
    encoder_re_pub = rx.Publisher(TwistWithCovarianceStamped, '/encoder/filtered', qos_pubber)

    ## Subscribers ##
    @rx.Subscriber(Bool, '/lli/remote/override', qos_subber)
    def override_sub(self, override_msg):
        self.override = override_msg.data

    @rx.Subscriber(Int8, '/lli/remote/throttle', qos_subber)
    def throttle_sub(self, rm_throttle_msg):
        if rm_throttle_msg.data >= 15:
            self.rm_throttle = 1
        elif rm_throttle_msg.data <= -15:
            self.rm_throttle = -1

    @rx.Subscriber(Int8, '/lli/ctrl/throttle', qos_subber)
    def ctrl_throttle_sub(self, ctrl_throttle_msg):
        if ctrl_throttle_msg.data >= 15:
            self.ctrl_throttle = 1
        elif ctrl_throttle_msg.data <= -15:
            self.ctrl_throttle = -1

    @rx.Subscriber(TwistWithCovarianceStamped, '/lli/sensor/encoders', qos_subber)
    def encoder_sub(self, encoder_msg):
        current_time = encoder_msg.header.stamp.sec + encoder_msg.header.stamp.nanosec * 1e-9

        # 如果是第一次回调，直接记录时间，不计算dt
        if getattr(self, "encoder_time", None) is None:
            self.encoder_time = current_time
            self.get_logger().info("First encoder message received, initializing timestamp")
            return

        # 计算时间差
        T = current_time - self.encoder_time
        self.encoder_time = current_time

        # 从消息中读取左右轮ticks（假设linear.y是左，linear.z是右）
        L = encoder_msg.twist.twist.linear.y
        R = encoder_msg.twist.twist.linear.z

        # 检查T是否有效
        if T <= 0:
            self.get_logger().warn(f"Invalid dt={T:.6f}, skipping")
            return
 
        # 调用转换函数
        v, w = self.ticks_to_twist(L, R, T)

        encoder_msg.twist.twist.angular.y = w
        encoder_msg.twist.twist.angular.z *= -1
        # encoder_msg.twist.twist.angular.z = w
 
        if self.override is None:
            pass
        else:
            if self.override:
                encoder_msg.twist.twist.linear.x = abs(encoder_msg.twist.twist.linear.x) * self.rm_throttle
                encoder_msg.twist.twist.angular.z *= self.rm_throttle
            else:
                encoder_msg.twist.twist.linear.x = abs(encoder_msg.twist.twist.linear.x) * self.ctrl_throttle
                encoder_msg.twist.twist.angular.z *= self.ctrl_throttle
        
        self.encoder_re_pub.publish(encoder_msg)

    def on_startup(self):
        pass

    def ticks_to_twist(self, ticks_L, ticks_R, dt):
        r = 0.055   # wheel radius [m]
        N = 40      # ticks per revolution
        b = 0.2     # track width [m]

        ds_L = 2 * 3.1415926 * r * (ticks_L / N)
        ds_R = 2 * 3.1415926 * r * (ticks_R / N)

        v_x = (ds_R + ds_L) / (2.0 * dt)
        omega_z = (ds_R - ds_L) / (b * dt)
        return v_x, omega_z


if __name__ == '__main__':
    encoder_filter.main()
