from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    slam_delay = LaunchConfiguration('slam_delay')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # SLAM Toolbox node
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'slam_delay',
            default_value='5.0',
            description='Delay in seconds before starting SLAM node'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([
                get_package_share_directory('svea_localization'),
                'params',
                'slam_sync.yaml'
            ]),
            description='Path to SLAM parameters file'
        ),
        
        # Launch with delay
        TimerAction(
            period=slam_delay,
            actions=[slam_node]
        )
    ])