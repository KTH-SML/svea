#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this

@launch_this
def main(
    map_pkg: str = 'svea_core',
    map_name: str = 'sml',
    use_foxglove: bool = True,
):
    bl = BetterLaunch()

    # Start map server
    bl.node("nav2_map_server", "map_server",
            name="map_server",
            params=dict(yaml_filename=bl.find(map_pkg, f"{map_name}.yaml"),
                        use_sim_time=False,
                        topic_name="map"))

    bl.node("nav2_lifecycle_manager", "lifecycle_manager",
            name="lifecycle_manager",
            params=dict(node_names=str(["map_server"]),
                        autostart=True))
    
    if use_foxglove:
        bl.include("foxglove_bridge", "foxglove_bridge_launch.xml",
                   port=8765)
