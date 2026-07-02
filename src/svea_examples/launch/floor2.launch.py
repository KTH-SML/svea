#! /usr/bin/env python3
from better_launch import BetterLaunch, launch_this

MAP_NAME = "floor2"

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

    if not is_sim:

        # Start SVEA in real-world mode
        bl.include("svea_core", "svea.launch.py",
                   is_sim=is_sim, # = False
                   map_name=MAP_NAME,
                   initial_pose_x=initial_pose_x,
                   initial_pose_y=initial_pose_y,
                   initial_pose_a=initial_pose_a)

        # The SVEA launch system is built to be compatible with multiple SVEAs running simultaneously.
        # Default name is "self", so to add the pure_pursuit node we need to namespace accordingly.
        with bl.group("self"):
        
            bl.node("svea_examples", "pure_pursuit.py",
                    name="pure_pursuit",
                    params={'points': points})

    if is_sim:
        # Start two SVEAs (svea_a and svea_b) in simulation, each with its own pure_pursuit node

        INITIAL_POSES = {
            "svea_a": (initial_pose_x, initial_pose_y, initial_pose_a),
            "svea_b": (0.0, 0.0, 0.0),
        }

        for name, (init_x, init_y, init_a) in INITIAL_POSES.items():
            
            bl.include("svea_core", "svea.launch.py",
                       name=name,
                       is_sim=is_sim,
                       is_indoor=True,
                       map_name=MAP_NAME,
                       initial_pose_x=init_x,
                       initial_pose_y=init_y,
                       initial_pose_a=init_a)

            # Add namespace to pure_pursuit node so that it can be run for each SVEA independently
            with bl.group(name):

                bl.node("svea_examples", "pure_pursuit.py",
                        name="pure_pursuit",
                        params={
                            # "points": points,
                            "localization/base_frame": f"{name}/base_link",
                        })

    bl.include("svea_core", "map_and_foxglove.launch.py",
               map_name=MAP_NAME,
               use_foxglove=use_foxglove)
