#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this

@launch_this
def main(
    lidar_ip: str = "192.168.0.10",
    lidar_frame: str = "laser",
    lidar_calibrate_time: bool = False,
    lidar_publish_intensity: bool = True,
    lidar_publish_multiecho: bool = False,
    lidar_angle_min: float = -2.355,
    lidar_angle_max: float = 2.355
):
    bl = BetterLaunch()

    bl.node("urg_node", "urg_node_driver",
            name="hokyou_lidar",
            params=dict(ip_address=lidar_ip,
                        laser_frame_id=lidar_frame,
                        calibrate_time=lidar_calibrate_time,
                        publish_intensity=lidar_publish_intensity,
                        publish_multiecho=lidar_publish_multiecho,
                        angle_min=lidar_angle_min,
                        angle_max=lidar_angle_max,
                        default_user_latency=0.01,
                        skip=1))
