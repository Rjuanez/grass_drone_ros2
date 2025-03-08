import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_project_gazebo = get_package_share_directory('grass_drone_gazebo')
    pkg_project_bringup = get_package_share_directory('grass_drone_bringup')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'ArucoWorld_takeoff.sdf'
        ])}.items(),
    )


    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_drone_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    #Camera(sky_cam) ROS2_Bridge 
    ros_gz_image_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/sky_cam@sensor_msgs/msg/Image@gz.msgs.Image"]
    )


    return LaunchDescription([
        gz_sim,
        bridge,
        ros_gz_image_bridge,

    ])
