#!/usr/bin/env python3
from pyrtcm import RTCMReader, RTCMMessage, datadesc
from serial import Serial, SerialException

from pyubx2 import (
    UBXReader,
    SET,
    UBXMessage,
    protocol,
    NMEA_PROTOCOL,  # 1
    UBX_PROTOCOL,  # 2
    RTCM3_PROTOCOL,  # 4
)
import rospy
from threading import Thread
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Sentence
from std_msgs.msg import Float64
from mavros_msgs.msg import RTCM

__author__ = "Mustafa Al-Janabi"
__email__ = "musaj@kth.se"
__license__ = "MIT"
__copyright__ = "Copyright 2023, Mustafa Al-Janabi"


class RTKReceiverManager:
    RATE = 30

    def __init__(self):
        self.device = rospy.get_param("~device")
        self.baud = rospy.get_param("~baud")
        self.frame_id = rospy.get_param("~gps_frame")
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

        self.nmea_pub = rospy.Publisher("/ntrip_client/nmea", Sentence, queue_size=10)
        self.fix_pub = rospy.Publisher("fix", NavSatFix, queue_size=10)
        self.heading_motion_pub = rospy.Publisher(
            "heading_motion", Float64, queue_size=10
        )
        self.heading_vehicle_pub = rospy.Publisher(
            "heading_vehicle", Float64, queue_size=10
        )
        self.headingAcc_pub = rospy.Publisher(
            "heading_accuracy", Float64, queue_size=10
        )
        self.speed_pub = rospy.Publisher("speed", Float64, queue_size=10)
        self.speedAcc_pub = rospy.Publisher("speed_accuracy", Float64, queue_size=10)
        self.magDec_pub = rospy.Publisher(
            "magnetic_declination", Float64, queue_size=10
        )
        self.magDecAcc_pub = rospy.Publisher(
            "magnetic_declination_accuracy", Float64, queue_size=10
        )
        self.rate = rospy.Rate(self.RATE)
        self.setup_receiver()
        self.nav_sat_fix_msg = NavSatFix()
        self.start_serial_read()
        # Subscribe to RTCM from NTRIP Client
        rospy.Subscriber("/ntrip_client/rtcm", RTCM, self._handle_rtcm_cb)

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
        # CFG-MSG-NAV-COV set rateUSB to 1
        self.set_config(0x01, 0x36, rateUSB=1)
        # CFG-MSG-RXM-RTCM set rateUSB to 1
        self.set_config(0x02, 0x32, rateUSB=1)

    @staticmethod
    def load_param(name, value=None):
        if value is None:
            assert rospy.has_param(name), f'Missing parameter "{name}"'
        return rospy.get_param(name, value)

    def start_serial_read(self):
        Thread(target=self._read_serial_handler, args=()).start()

    def _handle_rtcm_cb(self, msg):
        raw_rtcm = msg.data
        self.serial.write(raw_rtcm)

    def _read_serial_handler(self):
        while not rospy.is_shutdown():
            raw_msg, parsed_msg = self.ubx_reader.read()
            msg_protocol = protocol(raw_msg)
            if msg_protocol == UBX_PROTOCOL:
                self.nav_sat_fix_msg.header.stamp = rospy.Time().now()
                self.nav_sat_fix_msg.header.frame_id = self.frame_id
                if parsed_msg.identity == "NAV-PVT":
                    # Understanding the PVT message
                    # https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavPVT.msg
                    # and
                    # https://github.com/semuconsulting/pyubx2/blob/master/src/pyubx2/ubxtypes_get.py

                    # Only use Ok messages
                    if parsed_msg.gnssFixOk:
                        lon = parsed_msg.lon
                        lat = parsed_msg.lat
                        height = parsed_msg.height / 1e6  # [m]
                        horz_acc = parsed_msg.hAcc / 1e3  # [cm]
                        vert_acc = parsed_msg.vAcc / 1e3  # [cm]
                        fix_type = parsed_msg.fixType

                        self.nav_sat_fix_msg.latitude = parsed_msg.lat
                        self.nav_sat_fix_msg.longitude = parsed_msg.lon
                        self.nav_sat_fix_msg.altitude = parsed_msg.height / 1e6  # [m]

                        print(
                            lat,
                            lon,
                            height,
                            "m",
                            horz_acc,
                            "cm",
                            vert_acc,
                            "cm",
                            "fix",
                            fix_type,
                        )

                        # Publish Speed
                        self.speed_pub.publish(
                            Float64(parsed_msg.gSpeed * 1e-3)
                        )  # [m/s] convert from mm/s to m/s
                        self.speedAcc_pub.publish(
                            Float64(parsed_msg.sAcc * 1e-3)
                        )  # [m/s] convert from mm/s to m/s
                        # Publish Heading
                        self.heading_motion_pub.publish(
                            Float64(parsed_msg.headMot * 1e-5)
                        )  # [deg]
                        self.heading_vehicle_pub.publish(
                            Float64(parsed_msg.headVeh * 1e-5)
                        )  # [deg]
                        self.headingAcc_pub.publish(
                            Float64(parsed_msg.headAcc * 1e-5)
                        )  # [deg]
                        # Publish magDec
                        self.magDec_pub.publish(
                            Float64(parsed_msg.magDec * 1e-2)
                        )  # [deg]
                        self.magDecAcc_pub.publish(
                            Float64(parsed_msg.magAcc * 1e-2)
                        )  # [deg]

                elif parsed_msg.identity == "NAV-COV":
                    # NAV-COV is always sent directly after NAV-PVT, publish upon NAV-COV arrival
                    # Building a covariance matrix in the ENU frame
                    # [EE EN EU
                    #  NE NN NU
                    #  UE UN UU]
                    # However ZED-F9P gives us NED so we use the following equivalent form
                    # [ EE  EN -ED
                    #   NE  NN -ND
                    #  -DE -DN  DD]

                    ee = parsed_msg.posCovEE
                    ne = parsed_msg.posCovNE
                    ed = parsed_msg.posCovED
                    nn = parsed_msg.posCovNN
                    nd = parsed_msg.posCovND
                    dd = parsed_msg.posCovDD
                    position_covariance = [ee, ne, -ed, ne, nn, -nd, -ed, -nd, dd]
                    self.nav_sat_fix_msg.position_covariance = position_covariance
                    # Publish fix data
                    self.fix_pub.publish(self.nav_sat_fix_msg)
                    # Reset fix message after publishing
                    self.nav_sat_fix_msg = NavSatFix()
            if msg_protocol == NMEA_PROTOCOL:
                try:
                    nmea_str = raw_msg.decode("ascii")
                    nmea_sentence_msg = Sentence()
                    nmea_sentence_msg.sentence = nmea_str
                    nmea_sentence_msg.header.frame_id = self.frame_id
                    nmea_sentence_msg.header.stamp = rospy.Time().now()
                    self.nmea_pub.publish(nmea_sentence_msg)

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
