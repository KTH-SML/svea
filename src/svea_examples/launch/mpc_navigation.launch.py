from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import EqualsSubstitution 

def generate_launch_description():
    # Launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='sml',
        description='Map name'
    )
    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='true',
        description='Whether to use simulation'
    )
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='true',
        description='Whether to use Foxglove'
    )
    mpc_mode_arg = DeclareLaunchArgument(
        'mpc_mode',
        default_value='goal_position',
        description='MPC mode: goal_position or path_tracking'
    )
    svea_mocap_name_arg = DeclareLaunchArgument(
        'svea_mocap_name',
        default_value='svea7',
        description='Name of SVEA vehicle'
    )

    # Path substitutions
    map_path = PathJoinSubstitution([
        FindPackageShare('svea_core'),
        'maps',
        [LaunchConfiguration('map'), '.yaml']
    ])

    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_path,
            'use_sim_time': False,
            'topic_name': 'map'
        }]
    )

    lifecycle_manager = Node(
        package='lifecycle_manager',
        executable='nav2_lifecycle_manager',
        name='lifecycle_manager',
        parameters=[{
            'node_names': ['map_server'],
            'autostarts': True,
        }]
    )

    #Micro-ROS agent for SVEA

    import glob
    device_path = glob.glob('/dev/serial/by-id/*SVEA-LLI*')
    device = device_path[0] if device_path else '/dev/ttyUSB0'

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=[
            'serial',
            '--dev', device,
            '-b', '1000000'
        ],
        output='screen'
    )

    # Hardware interface (only when not in simulation)
    hardware_interface = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('is_sim')),
        actions=[
            micro_ros_agent,
        ]
    )

    # Simulation (only when in simulation)
    simulation = GroupAction(
        condition=IfCondition(LaunchConfiguration('is_sim')),
        actions=[
            IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare('svea_core'),
                    'launch/simulation.xml'
                ]))
        ]
    )

    # Foxglove bridge
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_foxglove')),
        parameters=[{
            'port': 8765,
            'use_sim_time': False
        }]
    )

    # Visualization
    visualization = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('svea_core'),
            'launch/svea_vizualization.launch.py'
        ])
    )
    
    # MPC Goal Position Mode
    mpc_goal = Node(
        package='svea_examples',
        executable='mpc.py',
        name='mpc',
        output='screen',
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('mpc_mode'),'goal_position')
        ),
        parameters=[
            {
                'use_rviz': LaunchConfiguration('use_foxglove'),
                'is_sim': LaunchConfiguration('is_sim'),
                'mpc_freq': 10,
                'target_speed': 0.3,
                'delta_s': 5,
                'svea_mocap_name': LaunchConfiguration('svea_mocap_name'),
                'mpc_config_ns': 'mpc'
            },
            PathJoinSubstitution([
                FindPackageShare('svea_core'),
                'params','mpc_default.yaml'
            ])
        ]
    )

    # MPC Path Tracking Mode
    mpc_path_tracking = Node(
        package='svea_examples',
        executable='mpc_path_tracking.py',
        name='mpc',
        output='screen',
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('mpc_mode'),'path_tracking')
        ),
        parameters=[
            {
                'use_rviz': LaunchConfiguration('use_foxglove'),
                'is_sim': LaunchConfiguration('is_sim'),
                'mpc_freq': 10,
                'target_speed': 1.0,
                'circle_radius': 1.5,
                'circle_center_x': 0.0,
                'circle_center_y': 0.0,
                'svea_mocap_name': LaunchConfiguration('svea_mocap_name'),
                'mpc_config_ns': 'mpc'
            },
            PathJoinSubstitution([
                FindPackageShare('svea_core'),
                'params','mpc_default_path_tracking.yaml'
            ])
        ]
    )

    return LaunchDescription([
        map_arg,
        is_sim_arg,
        use_foxglove_arg,
        mpc_mode_arg,
        svea_mocap_name_arg,
        # map_server,
        # lifecycle_manager,
        # hardware_interface,
        # simulation,
        # foxglove_bridge,
        # visualization,
        mpc_goal,
        mpc_path_tracking,
    ])