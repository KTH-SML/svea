import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
<<<<<<< HEAD
<<<<<<< HEAD
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
=======
from launch.substitutions import LaunchConfiguration
>>>>>>> 9585fc8 (Teleop control in simulation with teleop_twist_keyboard added)
=======
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
>>>>>>> 392cd10 (multi model displacement with namespace in foxglove fixed)
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
<<<<<<< HEAD
<<<<<<< HEAD
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

=======

    urdf_file_name = 'svea.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('svea_core'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
>>>>>>> 9585fc8 (Teleop control in simulation with teleop_twist_keyboard added)
=======
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

>>>>>>> 392cd10 (multi model displacement with namespace in foxglove fixed)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 392cd10 (multi model displacement with namespace in foxglove fixed)
        DeclareLaunchArgument(
            'vehicle_name',
            default_value='',
            description='Vehicle name for namespacing'),
<<<<<<< HEAD
=======
>>>>>>> 9585fc8 (Teleop control in simulation with teleop_twist_keyboard added)
=======
>>>>>>> 392cd10 (multi model displacement with namespace in foxglove fixed)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 392cd10 (multi model displacement with namespace in foxglove fixed)
            parameters=[{'use_sim_time': use_sim_time, 
                         'robot_description': robot_description,}]),
        Node(
            package='svea_core',
            executable='model_viz.py',
            name='svea_model',
=======
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='svea_core',
<<<<<<< HEAD
            executable='viz_util.py',
            name='viz_util',
>>>>>>> 9585fc8 (Teleop control in simulation with teleop_twist_keyboard added)
=======
            executable='model_viz.py',
            name='svea_model',
>>>>>>> 146db9c (marker placer complete)
            output='screen'),
    ])