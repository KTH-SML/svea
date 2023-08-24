#!/usr/bin/env python3
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
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nmea_msgs.msg import Sentence
from std_msgs.msg import Float64
from mavros_msgs.msg import RTCM

__author__ = "Mustafa Al-Janabi"
__email__ = "musaj@kth.se"
__license__ = "MIT"
__copyright__ = "Copyright 2023, Mustafa Al-Janabi"


# MAP dynamic model string to corresponding number
# source https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/CfgNAV5.msg
DYN_MODEL_MAP = {
    "portable": 0,
    "stationary": 1,    
    "pedestrian": 3,
    "automotive": 4,
    "sea": 5,
    "airborne_1g": 6,  # Airborne with <1g Acceleration
    "airborne_2g": 7,  # Airborne with <2g Acceleration
    "airborne_4g": 8,  # Airborne with <4g Acceleration
    "wrist_watch": 9,
    "bike": 10,
}
# MAP UBlox GPS quality to NAVSatMessage Quality
# from https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/NavPVT.msg
# and https://github.com/ros-drivers/nmea_navsat_driver/blob/b168f499349e5f26e3aed9982deb44ed2f41f8e2/src/libnmea_navsat_driver/driver.py#L108-L149
GPS_QUALITIES = {
    # Unknown
    -1: NavSatStatus.STATUS_NO_FIX,
    # Invalid
    0: NavSatStatus.STATUS_NO_FIX,
    # SPS
    1: NavSatStatus.STATUS_FIX,
    # DGPS
    2: NavSatStatus.STATUS_SBAS_FIX,  # Signal from only 3 SVs, constant altitude assumed
    # DGPS
    3: NavSatStatus.STATUS_SBAS_FIX,
    # RTK Fix
    4: NavSatStatus.STATUS_GBAS_FIX,  # GNSS + Dead reckoning
    # RTK Float
    5: NavSatStatus.STATUS_GBAS_FIX,  # Time only fix (High precision devices)
}


class RTKManager:
    RATE = 50

    def __init__(self):
        # Read parameters
        self.device = rospy.get_param("~device", "ttyACM0")
        self.baud = rospy.get_param("~baud", 250000)
        self.frame_id = rospy.get_param("~gps_frame", "gps")
        self.dynamic_model = rospy.get_param("~dynamic_model", "portable")
        # Setup ROS Rate
        self.rate = rospy.Rate(self.RATE)
        #  Open serial port
        try:
            self.serial = Serial(self.device, self.baud, timeout=3)
        except SerialException as ex:
            rospy.logfatal(
                "Could not open serial port: I/O error({0}): {1}".format(
                    ex.errno, ex.strerror
                )
            )
        # Initialise the ubx reader
        self.ubx_reader = UBXReader(
            self.serial, protfilter=UBX_PROTOCOL + NMEA_PROTOCOL
        )  # 1:NEMA 2:UBX 3:NMEA+UBX 4:RTCM 7: NMEA+UBX+RTCM
        # Create publishers
        self._init_pub()
        # Create subscriber
        self._init_sub()
        # Configure the receiver
        self.setup_receiver()
        # Start reading from serial port and parse messages using ubx_reader
        self.start_serial_read()

    def _init_pub(self):
        """Initializes publishers for necessary and sufficient topics"""
        # Nmea message which get sent to virtual NTRIP servers which give correction message from closes base station based on own location
        self.nmea_pub = rospy.Publisher("/ntrip_client/nmea", Sentence, queue_size=10)
        # Publish the satellite fix
        self.fix_pub = rospy.Publisher("fix", NavSatFix, queue_size=10)
        # Heading of 2-D motion in [deg]
        self.heading_motion_pub = rospy.Publisher(
            "heading_motion", Float64, queue_size=10
        )
        # Heading of vehicle in 2-D in [deg]
        self.heading_vehicle_pub = rospy.Publisher(
            "heading_vehicle", Float64, queue_size=10
        )
        # Combined heading accuracy of vehicle and motion headings in [deg]
        self.headingAcc_pub = rospy.Publisher(
            "heading_accuracy", Float64, queue_size=10
        )
        # Ground speed (2-D) in [m/s]
        self.speed_pub = rospy.Publisher("speed", Float64, queue_size=10)
        # Estimate of ground speed accuracy in [m/s]
        self.speedAcc_pub = rospy.Publisher("speed_accuracy", Float64, queue_size=10)
        # Magnetic declination in [deg]
        self.magDec_pub = rospy.Publisher(
            "magnetic_declination", Float64, queue_size=10
        )
        # Accuracy of magnetic declination in [deg]
        self.magDecAcc_pub = rospy.Publisher(
            "magnetic_declination_accuracy", Float64, queue_size=10
        )

    def _init_sub(self):
        """Initialize subscribers"""
        # Subscribe to RTCM correction messages from NTRIP Client
        rospy.Subscriber("/ntrip_client/rtcm", RTCM, self._handle_rtcm_cb)

    def set_config(self, msgClass, msgID, **kwargs):
        """Utility function which write a configuration message to receiver and awaits an acknowledgement."""
        cfg = UBXMessage(
            "CFG", "CFG-MSG", SET, msgClass=msgClass, msgID=msgID, **kwargs
        )
        self.serial.write(cfg.serialize())
        _, parsed_msg = self.ubx_reader.read()
        assert (
            parsed_msg.identity == "ACK-ACK"
        ), f"Failed to set CFG-MSG for msgClass:{msgClass} msgID:{msgID}"

    def set_dynamic_model(self, model):
        # CFG-MSG-NAV5 set dynModel (dynamic Model) to model
        # To understand the CFG-NAV5 msg https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/CfgNAV5.msg
        # and https://github.com/semuconsulting/pyubx2/blob/935f678a78a1038860d07aa64e600505bdc7ac00/src/pyubx2/ubxtypes_get.py#L566C6-L600
        if DYN_MODEL_MAP.get(model, None) is None:
            rospy.logwarn(
                f'Invalid Dynamic Model Provided: {self.model}. Supported models are {", ".join(DYN_MODEL_MAP.keys())}.'
            )
        else:
            cfg = UBXMessage(
                "CFG",
                "CFG-NAV5",
                SET,
                msgClass=0x06,
                msgID=0x24,
                dynModel=DYN_MODEL_MAP.get(model, 0),
                dyn=1,  # MASK to update only dynamic model
            )
            self.serial.write(cfg.serialize())
            _, parsed_msg = self.ubx_reader.read()
            assert (
                parsed_msg.identity == "ACK-ACK"
            ), f"Failed to set CFG-NAV5 for dynamic model:{model}."

    def setup_receiver(self):
        # CFG-MSG-NAV-STATUS set rateUSB to 0
        self.set_config(0x01, 0x03, rateUSB=0)
        # CFG-MSG-NAV-PVT set rateUSB to 1
        self.set_config(0x01, 0x07, rateUSB=1)
        # CFG-MSG-NAV-COV set rateUSB to 1
        self.set_config(0x01, 0x36, rateUSB=1)
        # CFG-MSG-RXM-RTCM set rateUSB to 1
        self.set_config(0x02, 0x32, rateUSB=1)
        # CFG-NAV5 set dynModel to self.dynamic_model
        self.set_dynamic_model(self.dynamic_model)

    def start_serial_read(self):
        """Start new thread which reads from the serial port and handles the incoming messages from the receiver."""
        self.nav_sat_fix_msg = NavSatFix()
        Thread(target=self._read_serial_handler, args=()).start()

    def _handle_rtcm_cb(self, msg):
        """Callback which listens to RTCM messages from NTRIP clients and writes them to the receiver."""
        raw_rtcm = msg.data
        self.serial.write(raw_rtcm)

    def _read_serial_handler(self):
        """Manager of all incoming messages from Serial port, reads from serial port at self.RATE [Hz]"""
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
                        # Comment out the following to debug with a print
                        # lon = parsed_msg.lon
                        # lat = parsed_msg.lat
                        # height = parsed_msg.height / 1e3  # [m]
                        # horz_acc = parsed_msg.hAcc / 1e3  # [m]
                        # vert_acc = parsed_msg.vAcc / 1e3  # [m]
                        # fix_type = parsed_msg.fixType
                        # print(
                        #     parsed_msg.iTOW,
                        #     lat,
                        #     lon,
                        #     height,
                        #     "m",
                        #     parsed_msg.hAcc / 10,
                        #     "cm",
                        #     parsed_msg.vAcc / 10,
                        #     "cm",
                        #     "fix",
                        #     fix_type,
                        # )

                        self.nav_sat_fix_msg.latitude = parsed_msg.lat  # [deg]
                        self.nav_sat_fix_msg.longitude = parsed_msg.lon  # [deg]
                        self.nav_sat_fix_msg.altitude = parsed_msg.height / 1e3  # [m]
                        # set status status
                        self.nav_sat_fix_msg.status.status = GPS_QUALITIES.get(
                            parsed_msg.fixType,
                            self.nav_sat_fix_msg.status.STATUS_NO_FIX,
                        )
                        # Set status service to GPS
                        self.nav_sat_fix_msg.status.service = (
                            self.nav_sat_fix_msg.status.SERVICE_GPS
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
                            # Float64(parsed_msg.headMot * 1e-5)
                            Float64(parsed_msg.headMot)
                        )  # [deg]
                        self.heading_vehicle_pub.publish(
                            # Float64(parsed_msg.headVeh * 1e-5)
                            Float64(parsed_msg.headVeh)
                        )  # [deg]
                        self.headingAcc_pub.publish(
                            # Float64(parsed_msg.headAcc * 1e-5)
                            Float64(parsed_msg.headAcc)
                        )  # [deg]
                        # Publish magDec
                        self.magDec_pub.publish(
                            # Float64(parsed_msg.magDec * 1e-2)
                            Float64(parsed_msg.magDec)
                        )  # [deg]
                        self.magDecAcc_pub.publish(
                            # Float64(parsed_msg.magAcc * 1e-2)
                            Float64(parsed_msg.magAcc)
                        )  # [deg]

                # https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/CfgNAV5.msg
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

                    # In some other Ublox GPS library the covariance is estimated by
                    # computing the diagonal elements using the hAcc, and vAcc values in the
                    # PVT message. In such case we convert the values to meters and raise by 2 to get
                    # an approximation of the
                    # positive_covariance[0] = (pvt_msg.hAcc / 1e3) ** 2
                    # positive_covariance[4] = (pvt_msg.hAcc / 1e3) ** 2
                    # positive_covariance[4] = (pvt_msg.vAcc / 1e3) ** 2
                    # see https://github.com/KumarRobotics/ublox/blob/4f107f3b82135160a1aca3ef0689fd119199bbef/ublox_gps/src/node.cpp#LL779C1-L787C62
                    # However, using the NAV-COV message gives more accurate results
                    # and experimentation shows that the diagonal approximation methods gives
                    # (EE + NN) is approx. (pvt_msg.hAcc / 1e3) ** 2
                    # Ie. the east-east and north-north component add up to the approximated
                    # diagonal values horizontally.
                    # DD is almost the same as (pvt_msg.vAcc / 1e3) ** 2 which
                    # means that both methods which approximate the vertical covariance are in agreements.
                    # Of course, the additional benefit of using the NAV-COV message is that we also get
                    # the covariance values for the cross-terms.

                    ee = parsed_msg.posCovEE
                    ne = parsed_msg.posCovNE
                    ed = parsed_msg.posCovED
                    nn = parsed_msg.posCovNN
                    nd = parsed_msg.posCovND
                    dd = parsed_msg.posCovDD
                    position_covariance = [ee, ne, -ed, ne, nn, -nd, -ed, -nd, dd]
                    # Set covariance
                    self.nav_sat_fix_msg.position_covariance = position_covariance
                    # Set covariance type to known
                    self.nav_sat_fix_msg.position_covariance_type = (
                        self.nav_sat_fix_msg.COVARIANCE_TYPE_KNOWN
                    )
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
    rospy.init_node("rtk_manager", anonymous=False)
    rtk_manager = RTKManager()
    rospy.spin()
