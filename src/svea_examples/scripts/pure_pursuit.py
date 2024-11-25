#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


class BatteryChargerMonitor:

    RATE = 1  # Hz

    def __init__(self):

        ## Initialize node

        rospy.init_node("battery_charger_monitor")

        ## Parameters

        self.log_format = load_param("~log_format", "{:.3f}")

        ## Subscribers

        # Battery topics
        rospy.Subscriber("/battery/current", Float32, self.battery_current_callback)
        rospy.Subscriber("/battery/voltage", Float32, self.battery_voltage_callback)
        rospy.Subscriber("/battery/shunt_voltage", Float32, self.battery_shunt_callback)

        # Charger topics
        rospy.Subscriber("/charger/current", Float32, self.charger_current_callback)
        rospy.Subscriber("/charger/voltage", Float32, self.charger_voltage_callback)
        rospy.Subscriber("/charger/shunt_voltage", Float32, self.charger_shunt_callback)

        ## Initialize state

        self.battery_current = 0.0
        self.battery_voltage = 0.0
        self.battery_shunt_voltage = 0.0

        self.charger_current = 0.0
        self.charger_voltage = 0.0
        self.charger_shunt_voltage = 0.0

        ## Log startup message

        rospy.loginfo("Battery and Charger Monitor initialized and running.")

    def run(self):
        rate = rospy.Rate(self.RATE)
        while self.keep_alive():
            self.spin()
            rate.sleep()

    def keep_alive(self):
        return not rospy.is_shutdown()

    def spin(self):
        rospy.loginfo(self.format_data())

    def format_data(self):
        return (
            f"Battery -> Current: {self.log_format.format(self.battery_current)} A, "
            f"Voltage: {self.log_format.format(self.battery_voltage)} V, "
            f"Shunt Voltage: {self.log_format.format(self.battery_shunt_voltage)} V\n"
            f"Charger -> Current: {self.log_format.format(self.charger_current)} A, "
            f"Voltage: {self.log_format.format(self.charger_voltage)} V, "
            f"Shunt Voltage: {self.log_format.format(self.charger_shunt_voltage)} V"
        )

    ## Callback functions for battery

    def battery_current_callback(self, msg):
        self.battery_current = msg.data

    def battery_voltage_callback(self, msg):
        self.battery_voltage = msg.data

    def battery_shunt_callback(self, msg):
        self.battery_shunt_voltage = msg.data

    ## Callback functions for charger

    def charger_current_callback(self, msg):
        self.charger_current = msg.data

    def charger_voltage_callback(self, msg):
        self.charger_voltage = msg.data

    def charger_shunt_callback(self, msg):
        self.charger_shunt_voltage = msg.data


if __name__ == "__main__":

    ## Start node ##

    monitor = BatteryChargerMonitor()
    monitor.run()