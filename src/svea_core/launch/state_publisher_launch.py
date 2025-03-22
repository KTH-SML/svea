from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare the launch arguments
    declare_map_arg = DeclareLaunchArgument(
        'map', default_value='sml',
        description='Map file to load'
    )
    declare_initial_pose_x_arg = DeclareLaunchArgument(
        'initial_pose_x', default_value='-2.65488696',
        description='Initial pose x'
    )
    declare_initial_pose_y_arg = DeclareLaunchArgument(
        'initial_pose_y', default_value='-1.64422277',
        description='Initial pose y'
    )
    declare_initial_pose_a_arg = DeclareLaunchArgument(
        'initial_pose_a', default_value='1.57',
        description='Initial pose a'
    )

    # Start map server
    map_server_node = Node(
        package='map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(
            get_package_share_directory('svea_core'), 'maps', LaunchConfiguration('map') + '.yaml')}]
    )

    # Start low-level interface
    serial_node = Node(
        package='rosserial_python',
        executable='serial_node.py',
        name='serial_node',
        parameters=[{'port': '/dev/ttyACM0', 'baud': 250000}]
    )

    # Start localization
    localize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('svea_sensors'),
                'launch',
                'localize.launch.py'
            )
        ),
        launch_arguments={
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
            'initial_pose_a': LaunchConfiguration('initial_pose_a')
        }.items()
    )

    # Start state publisher
    state_publisher_node = Node(
        package='svea_core',
        executable='state_publisher.py',
        name='state_publisher',
        output='screen'
    )

    return LaunchDescription([
        declare_map_arg,
        declare_initial_pose_x_arg,
        declare_initial_pose_y_arg,
        declare_initial_pose_a_arg,
        map_server_node,
        serial_node,
        localize_launch,
        state_publisher_node
    ])