This is my IITB Racing Trainee SLAM Module project.
Takes input from eufs_sim topics
Note: Source dv_msgs package before building this project
      Uses nvcc compiler

* All relevant topics to subsribe to in rviz are mentioned in the code.

* Motion Update part: Done
    Estimates position only using motion update, no measurement update step involved.
    Launch using: ros2 launch slam_project checkpoint1_3

* EKF part: Done, need to tweak parameters.
    Implements EKF
    Launch using: ros2 launch slam_project localization_EKF_SLAM.launch.py




Details:
Launch file:
    checkpoint1_3.launch.py
    Launches: localization_node for checkpoint 1 and 3

Nodes:
    localization_node: Does position estimation as given in checkpoint 1, and NNAssociation as given in checkpoint3
    localization_node_ekf: Implement EKF

Build command
colcon build --cmake-args -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.1/bin/nvcc