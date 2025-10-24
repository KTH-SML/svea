import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    vehicle_name = LaunchConfiguration('vehicle_name', default='')
    xacro_file_name = 'svea.urdf.xacro'
    xacro_file = os.path.join(
        get_package_share_directory('svea_core'),
        'urdf',
        xacro_file_name)
    namespace_with_slash = PythonExpression([
        "'", vehicle_name, "/' if '", vehicle_name, "' != '' else ''"
    ])
    robot_description = Command([
        'xacro ', xacro_file, ' namespace:=', namespace_with_slash
    ])


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'vehicle_name',
            default_value='',
            description='Vehicle name for namespacing'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 
                         'robot_description': robot_description,}]),
        Node(
            package='svea_core',
            executable='model_viz.py',
            name='svea_model',
            output='screen'),
    ])