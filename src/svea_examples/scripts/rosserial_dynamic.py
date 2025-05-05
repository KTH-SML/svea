#!/usr/bin/env python3
import os
import rospy
from rosserial_python.SerialClient import SerialClient
from serial import SerialException
from threading import Thread


def load_param(name, default=None):
    """
    Load a parameter from the parameter server with a default fallback.
    """
    return rospy.get_param(name, default)


class DynamicSerialManager:
    def __init__(self):
        rospy.init_node("dynamic_serial_manager")

        # Parameters
        self.baud_rate = load_param("~baud", 115200)
        self.rate = rospy.Rate(
            load_param("~scan_rate", 0.1)
        )  # Scanning interval in seconds

        # Active serial clients
        self.active_clients = {}

        rospy.loginfo("Dynamic Serial Manager initialized.")

    def run(self):
        """
        Main loop to monitor and manage serial devices.
        """
        rospy.loginfo("Starting dynamic serial management...")

        while not rospy.is_shutdown():
            self.monitor_devices()
            self.rate.sleep()

    def monitor_devices(self):
        """
        Detect connected devices and start or stop clients accordingly.
        """
        # Detect all /dev/ttyACM* devices
        detected_ports = {
            f"/dev/{dev}"
            for dev in os.listdir("/dev")
            if "ttyACM" in dev or "USB" in dev
        }

        # Start clients for new devices
        for port in detected_ports - self.active_clients.keys():
            self.start_client(port)

        # Stop clients for removed devices
        for port in self.active_clients.keys() - detected_ports:
            self.stop_client(port)

    def start_client(self, port):
        """
        Start a SerialClient for the specified port.
        """
        rospy.loginfo(f"Starting client for {port}")
        thread = Thread(target=self.run_client, args=(port,), daemon=True)
        thread.start()
        self.active_clients[port] = thread

    def stop_client(self, port):
        """
        Stop a SerialClient for the specified port.
        """
        rospy.loginfo(f"Stopping client for {port}")
        del self.active_clients[port]

    def run_client(self, port):
        """
        Run a SerialClient for a single port in a separate thread.
        """
        while not rospy.is_shutdown():
            rospy.loginfo(f"Connecting to {port} at {self.baud_rate} baud")
            try:
                client = SerialClient(port, self.baud_rate)
                client.run()
            except (SerialException, OSError):
                rospy.logwarn(f"Lost connection to {port}. Retrying...")
                continue
            except:
                rospy.loginfo(f"Shutting down client for {port}")
                break


if __name__ == "__main__":
    try:
        manager = DynamicSerialManager()
        manager.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Dynamic Serial Manager shutting down.")
