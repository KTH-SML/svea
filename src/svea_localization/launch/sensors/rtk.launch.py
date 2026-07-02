#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this

@launch_this
def main(
    ## RTK RECEIVER ARGUMENTS
    device: str = "/dev/ttyACM0",           # /dev/ttyS0 if connected via UART
    baud: int = 250000,                     # 38400 if connected via UART
    dynamic_model: str = "portable",
    ## NTRIP CLIENT ARGUMENTS (for swepos network rtk)
    host: str = "nrtk-swepos.lm.se",
    port: int = 80,                         # PORT 8500 is also valid
    authenticate: bool = True,
    mountpoint: str = "MSM_GNSS",
    username: str = "",
    password: str = ""
):
    
    bl = BetterLaunch()

    with bl.group("gps"):

        # Start RTK Manager Node
        bl.node("svea_localization", "rtk_manager.py",
                name="rtk_manager",
                params=dict(device=device,
                            baud=baud,
                            gps_frame="gps",
                            dynamic_model=dynamic_model))

    # Start NTRIP Client
    bl.include("ntrip_client", "ntrip_client_launch.py",
               namespace="gps",
               host=host,
               port=port,
               mountpoint=mountpoint,
               authenticate=authenticate,
               username=username,
               password=password)
