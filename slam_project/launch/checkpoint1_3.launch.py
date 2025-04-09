import launch
import launch_ros.actions
from launch.actions import ExecuteProcess
import os
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = "slam_project"
    rosbag_path = os.path.join(FindPackageShare(package_name).find(package_name), 'rosbag', 'rosbag_motion_update', 'rosbag2_2025_03_31-01_47_00_0.db3')
    return launch.LaunchDescription([
        # Starting localization node
        launch_ros.actions.Node(
            package=package_name,
            executable='localization_node',
            name='localization_node',
            output='screen',
        ),
    ])