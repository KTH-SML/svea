#! /usr/bin/env python3

import numpy as np
import math
import tf_transformations as tf
from svea_core.models.bicycle import Bicycle4DWithESC
from svea_core.interfaces import LocalizationInterface, ActuationInterface
try:
    from svea_mocap.mocap import MotionCaptureInterface
except ImportError:
    pass
from svea_core.controllers.mpc import MPC

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.clock import Clock
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray

from svea_core import rosonic as rx

qos_subber = QoSProfile(depth=10 # Size of the queue 
                        )

qos_pubber = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

class mpc_path_tracking(rx.Node):
    is_sim = rx.Parameter(True)
    state = rx.Parameter([-3.0, 0.0, 0.0, 0.0])  # x, y, yaw, velocity
    mpc_freq = rx.Parameter(10)  # Hz
    svea_mocap_name = rx.Parameter('svea7')
    target_speed = rx.Parameter(0.5)  # m/s
    circle_radius = rx.Parameter(5.0)  # m
    circle_center_x = rx.Parameter(0.0)  # m
    circle_center_y = rx.Parameter(0.0)  # m
    mpc_config_ns = rx.Parameter('/mpc')
    time_step = rx.Parameter(0.2)  # m
    prediction_horizon = rx.Parameter(5)

    