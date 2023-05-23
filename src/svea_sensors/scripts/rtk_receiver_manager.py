#!/usr/bin/env python3
from pyrtcm import RTCMReader, RTCMMessage, datadesc
from serial import Serial, SerialException

from pyubx2 import (
    UBXReader,
    UBX_CONFIG_DATABASE,
    SET,
    UBXMessage,
    GET,
    POLL,
    protocol,
    NMEA_PROTOCOL,  # 1
    UBX_PROTOCOL,  # 2
    RTCM3_PROTOCOL,  # 4
    ERR_IGNORE,
    ERR_RAISE,
)
import rospy
from threading import Thread
from sensor_msgs.msg import NavSatFix
from libnmea_navsat_driver.driver import RosNMEADriver

__author__ = "Mustafa Al-Janabi"
__email__ = "musaj@kth.se"
__license__ = "MIT"
__copyright__ = "Copyright 2023, Mustafa Al-Janabi"


class RTKReceiverManager:
    RATE = 20

    def __init__(self):
        self.device = rospy.get_param("~device")
        self.baud = rospy.get_param("~baud")
        try:
            self.serial = Serial(self.device, self.baud, timeout=3)
        except SerialException as ex:
            rospy.logfatal(
                "Could not open serial port: I/O error({0}): {1}".format(
                    ex.errno, ex.strerror
                )
            )
        self.ubx_reader = UBXReader(
            self.serial, protfilter=UBX_PROTOCOL + NMEA_PROTOCOL
        )  # 1:NEMA 2:UBX 3:NMEA+UBX 4:RTCM 7: NMEA+UBX+RTCM

        self.rate = rospy.Rate(self.RATE)
        self.setup_receiver()
        self.nmea_driver = RosNMEADriver()
        self.frame_id = self.nmea_driver.get_frame_id()
        self.start_read()

    def set_config(self, msgClass, msgID, **kwargs):
        cfg = UBXMessage(
            "CFG", "CFG-MSG", SET, msgClass=msgClass, msgID=msgID, **kwargs
        )
        self.serial.write(cfg.serialize())
        _, parsed_msg = self.ubx_reader.read()
        assert (
            parsed_msg.identity == "ACK-ACK"
        ), f"Failed to set CFG-MSG for msgClass:{msgClass} msgID:{msgID}"

    def setup_receiver(self):
        # CFG-MSG-NAV-STATUS set rateUSB to 0
        self.set_config(0x01, 0x03, rateUSB=0)
        # CFG-MSG-NAV-PVT set rateUSB to 1
        self.set_config(0x01, 0x07, rateUSB=1)
        # CFG-MSG-RXM-RTCM set rateUSB to 1
        self.set_config(0x02, 0x32, rateUSB=1)

    @staticmethod
    def load_param(name, value=None):
        if value is None:
            assert rospy.has_param(name), f'Missing parameter "{name}"'
        return rospy.get_param(name, value)

    def start_read(self):
        Thread(target=self._read_handler, args=()).start()

    def _read_handler(self):
        while not rospy.is_shutdown():
            raw_msg, parsed_msg = self.ubx_reader.read()
            msg_protocol = protocol(raw_msg)
            if msg_protocol == UBX_PROTOCOL:
                if parsed_msg.identity == "NAV-PVT":
                    nav_sat_fix_msg = NavSatFix()
                    # nav_sat_fix_msg.header.stamp = rospy.Time().now()
                    # nav_sat_fix_msg.latitude = parsed_msg.lat
                    # nav_sat_fix_msg.longitude = parsed_msg.lon
                    # nav_sat_fix_msg.altitude = parsed_msg.height
                    # print(nav_sat_fix_msg)
                    # lon = parsed_msg.lon
                    # lat = parsed_msg.lat
                    # height = parsed_msg.height
                    # horz_acc = parsed_msg.hAcc
                    # vert_acc = parsed_msg.vAcc
                    # print(lat, lon, height, horz_acc, vert_acc)

                    # print(parsed_msg)
            elif msg_protocol == NMEA_PROTOCOL:
                try:
                    nmea_str = raw_msg.decode("ascii")
                    print(nmea_str)
                    self.nmea_driver.add_sentence(nmea_str, self.frame_id)
                    # TODO add publisher of sentence to /ntrip_client/nmea
                    # TODO subscribe to RTCM message from /ntrip_client/RTCM
                    # TODO serial write thee RTCM message to receiver
                except UnicodeError as e:
                    rospy.logwarn(
                        "Skipped adding a NMEA sentence from serial device becuase it could not be decoded as an ASCII string. The bytes were {0}".format(
                            raw_msg
                        )
                    )

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("rtk_receiver_manager", anonymous=False)
    rtk_manager = RTKReceiverManager()
    rospy.spin()
    # r = rospy.Rate(20)
    # while not rospy.is_shutdown():
    #     print(rospy.Time().now())
    #     r.sleep()
