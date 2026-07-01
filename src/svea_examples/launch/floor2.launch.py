#! /usr/bin/env python3
from better_launch import BetterLaunch, launch_this

@launch_this
def main(
    is_sim: bool = True,
    use_foxglove: bool = True,
    initial_pose_x: float = -7.4,
    initial_pose_y: float = -15.4,
    initial_pose_a: float = +0.9,
    points: str = '[[-2.3, -7.1], [10.5, 11.7], [5.7,  15.0], [-7.0, -4.0]]',
):
    bl = BetterLaunch()

    MAP_NAME = "floor2"

    bl.include("svea_core", "map_and_foxglove.xml",
               map_name=MAP_NAME,
               use_foxglove=use_foxglove)

    if not is_sim:

        NAME = "self"

        # Start SVEA system
        bl.include("svea_core", "svea.xml",
                   name=NAME,
                   is_sim=is_sim,
                   initial_pose_x=initial_pose_x,
                   initial_pose_y=initial_pose_y,
                   initial_pose_a=initial_pose_a,
                   map=MAP_NAME)

        with bl.group(ns=NAME):

            # Start pure_pursuit
            bl.node("svea_examples", "pure_pursuit.py",
                    name="pure_pursuit",
                    params=dict(is_sim=is_sim,
                                points=points,
                                localization_base_frame=f"{NAME}/base_link")))

    if is_sim:
        # Start two SVEAs (svea_a and svea_b) in simulation, each with its own pure_pursuit node

        raise NotImplementedError("This launch file is not yet implemented for simulation mode. Please use the demo.launch.py file instead.")
        
        INITIAL_POSES = {
            "svea_a": (initial_pose_x, initial_pose_y, initial_pose_a),
            "svea_b": (0.0, 0.0, 0.0),
        }

        for name, (init_x, init_y, init_a) in INITIAL_POSES.items():
            
            bl.include("svea_core", "svea.xml",
                       name=name,
                       is_sim=is_sim,
                       is_indoor=True,
                       map_name=MAP_NAME,
                       initial_pose_x=init_x,
                       initial_pose_y=init_y,
                       initial_pose_a=init_a)

            with bl.group(ns=name):

                # Start pure_pursuit
                bl.node("svea_examples", "pure_pursuit.py",
                        name="pure_pursuit",
                        params={
                            "is_sim": is_sim,
                            "points": points,
                            "localization/base_frame": f"{name}/base_link",
                        })
