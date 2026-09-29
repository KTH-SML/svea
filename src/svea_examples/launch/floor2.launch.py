#! /usr/bin/env python3
from better_launch import BetterLaunch, launch_this

MAP_NAME = "floor2"
POINTS: list = [-2.3, -7.1, 10.5, 11.7, 5.7, 15.0, -8.0, -5.4]

@launch_this
def main(
    is_sim: bool = True,
    use_foxglove: bool = True,
    initial_pose_x: float = -7.4,
    initial_pose_y: float = -15.4,
    initial_pose_a: float = +0.9,
    target_velocity: float = 0.6,
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
        # Default name is "self", so to add the static_path_follower node we need to namespace accordingly.
        with bl.group("self"):
        
            bl.node("svea_examples", "static_path_follower.py",
                    name="static_path_follower",
                    params={'points': POINTS, 
                            'is_sim': is_sim,
                            'target_velocity': target_velocity})

    if is_sim:
        # Start two SVEAs (svea_a and svea_b) in simulation, each with its own static_path_follower node

        INITIAL_POSES = {
            "svea_a": (initial_pose_x, initial_pose_y, initial_pose_a),
            "svea_b": (0.0, 0.0, 0.0),
        }

        bl.node("nav2_map_server", "map_server",
                name="map_server",
                params=dict(yaml_filename=bl.find('svea_core', f"{MAP_NAME}.yaml"),
                            use_sim_time=False))

        for name, (init_x, init_y, init_a) in INITIAL_POSES.items():
            
            bl.include("svea_core", "svea.launch.py",
                       name=name,
                       is_sim=is_sim,
                       is_indoor=True,
                       map_name=MAP_NAME,
                       use_map=False,
                       initial_pose_x=init_x,
                       initial_pose_y=init_y,
                       initial_pose_a=init_a)

            # Add namespace to static_path_follower node so that it can be run for each SVEA independently
            with bl.group(name):

                bl.node("svea_examples", "static_path_follower.py",
                        name="static_path_follower",
                        params={
                            "points": POINTS,
                            "is_sim": is_sim,
                            "target_velocity": target_velocity,
                            "localization/base_frame": f"{name}/base_link",
                        })
    if use_foxglove:
        bl.include("foxglove_bridge", "foxglove_bridge_launch.xml")
