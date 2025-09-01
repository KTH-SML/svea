import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 获取参数文件路径
    config_dir = '/home/tokisaki_kurumi/Projects/svea/better_launch/src/svea_localization/params/slam_toolbox/'  # 替换为您的配置文件路径
    mapper_params_file = os.path.join(config_dir, 'mapper_params_online_async.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                mapper_params_file,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        )
    ])