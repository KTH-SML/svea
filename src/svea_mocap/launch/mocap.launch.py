#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this

def toargs(**kwds):
    args = []
    for k, v in kwds.items():
        args.append(f"--{k.replace('_', '-')}")
        args.append(str(v))
    return args

@launch_this
def main(              # For SML:
    map_x: float = 0., #    = 0.06,
    map_y: float = 0., #    = -0.06,
    map_a: float = 0., #    = 1.57,
    mocap_frame: str = "mocap",
    map_frame: str = "map",
):

    bl = BetterLaunch()

    bl.include("mocap_qualisys", "qualisys.launch.py",
               server_address="10.0.0.10",
               server_base_port="22222",
               fixed_frame_id=mocap_frame,
               publish_tf=True)

    bl.node("tf2_ros", "static_transform_publisher",
            name="broadcaster_mocap_map",
            cmd_args=toargs(x=map_x, y=map_y, z=0,
                            yaw=map_a, pitch=0, roll=0,
                            frame_id=mocap_frame,
                            child_frame_id=map_frame))

