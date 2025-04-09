import launch
import launch_ros.actions
from launch.actions import ExecuteProcess
import os
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = "slam_project"
    rosbag_path = "/home/siddheshsp022/RACING_TEAM/ROS_PROJECTS/slam_ws/src/slam_project/rosbag/rosbag_localisation-EKF/rosbag_localisation/rosbag2_2025_04_06-23_11_57_0.db3"
    return launch.LaunchDescription([
        # Starting localization node
        launch_ros.actions.Node(
            package=package_name,
            executable='localization_node_ekf',
            name='localization_node_ekf',
            output='screen',
        ),
    ])