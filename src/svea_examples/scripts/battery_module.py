#! /usr/bin/env python3

import rospy
from rospy import Publisher, Subscriber, Rate
from std_msgs.msg import Float32


def load_param(name, default=None):
    """
    Helper function to load ROS parameters with assertion.
    """
    if default is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, default)


def assert_points(pts):
    """
    Ensure that the provided points are in the correct format.
    """
    assert isinstance(
        pts, (list, tuple)
    ), "points is of wrong type, expected list or tuple"
    for xy in pts:
        assert isinstance(
            xy, (list, tuple)
        ), "points contain an element of wrong type, expected list of two values (x, y)"
        assert (
            len(xy) == 2
        ), "points contain an element of wrong type, expected list of two values (x, y)"
        x, y = xy
        assert isinstance(
            x, (int, float)
        ), "points contain a coordinate pair wherein one value is not a number"
        assert isinstance(
            y, (int, float)
        ), "points contain a coordinate pair wherein one value is not a number"


class BatteryChargerMonitor:
    """
    A ROS node that monitors battery and charger data published by an Arduino via rosserial.
    It processes the data and publishes additional information like State of Charge (SoC).
    """

    def __init__(self):
        ## Initialize node
        rospy.init_node("battery_charger_monitor", anonymous=True)

        ## Parameters
        self.serial_port = load_param("~serial_port", "/dev/ttyACM2")
        self.baud_rate = load_param("~baud_rate", 921600)

        self.battery_min_voltage = load_param("~battery_min_voltage", 9.0)  # in Volts
        self.battery_max_voltage = load_param("~battery_max_voltage", 12.6)  # in Volts

        ## Subscribers for Battery
        self.battery_shunt_voltage_sub = Subscriber(
            "/battery/shunt_voltage", Float32, self.battery_shunt_voltage_callback
        )
        self.battery_current_sub = Subscriber(
            "/battery/current", Float32, self.battery_current_callback
        )
        self.battery_bus_voltage_sub = Subscriber(
            "/battery/bus_voltage", Float32, self.battery_bus_voltage_callback
        )

        ## Subscribers for Charger
        self.charger_shunt_voltage_sub = Subscriber(
            "/charger/shunt_voltage", Float32, self.charger_shunt_voltage_callback
        )
        self.charger_current_sub = Subscriber(
            "/charger/current", Float32, self.charger_current_callback
        )
        self.charger_bus_voltage_sub = Subscriber(
            "/charger/bus_voltage", Float32, self.charger_bus_voltage_callback
        )

        ## Publisher for State of Charge
        self.soc_pub = Publisher("/battery/state_of_charge", Float32, queue_size=10)

        ## Initialize data variables
        self.battery_shunt_voltage = 0.0
        self.battery_current = 0.0
        self.battery_bus_voltage = 0.0

        self.charger_shunt_voltage = 0.0
        self.charger_current = 0.0
        self.charger_bus_voltage = 0.0

        ## Set loop rate
        self.rate = Rate(50)  # 2 Hz

        ## Start main loop
        self.main_loop()

    ## Callback functions for Battery
    def battery_shunt_voltage_callback(self, msg):
        self.battery_shunt_voltage = msg.data

    def battery_current_callback(self, msg):
        self.battery_current = msg.data

    def battery_bus_voltage_callback(self, msg):
        self.battery_bus_voltage = msg.data
        self.calculate_soc()

    ## Callback functions for Charger
    def charger_shunt_voltage_callback(self, msg):
        self.charger_shunt_voltage = msg.data

    def charger_current_callback(self, msg):
        self.charger_current = msg.data

    def charger_bus_voltage_callback(self, msg):
        self.charger_bus_voltage = msg.data

    def calculate_soc(self):
        """
        Calculate the State of Charge (SoC) based on bus voltage using a simple linear mapping.
        """
        voltage = self.battery_bus_voltage
        if voltage <= self.battery_min_voltage:
            soc = 0.0
        elif voltage >= self.battery_max_voltage:
            soc = 100.0
        else:
            soc = (
                (voltage - self.battery_min_voltage)
                / (self.battery_max_voltage - self.battery_min_voltage)
            ) * 100.0
        # Publish SoC
        soc_msg = Float32()
        soc_msg.data = soc
        self.soc_pub.publish(soc_msg)

    def main_loop(self):
        """
        Main loop to log data periodically.
        """
        try:
            while not rospy.is_shutdown():
                rospy.loginfo(
                    f"Battery - Shunt Voltage: {self.battery_shunt_voltage:.3f} V | "
                    f"Current: {self.battery_current:.3f} A | "
                    f"Bus Voltage: {self.battery_bus_voltage:.3f} V"
                )
                rospy.loginfo(
                    f"Charger - Shunt Voltage: {self.charger_shunt_voltage:.3f} V | "
                    f"Current: {self.charger_current:.3f} A | "
                    f"Bus Voltage: {self.charger_bus_voltage:.3f} V"
                )
                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass
        finally:
            rospy.loginfo("Shutting down BatteryChargerMonitor node.")


if __name__ == "__main__":
    try:
        BatteryChargerMonitor()
    except rospy.ROSInterruptException:
        pass
