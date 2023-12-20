# Robot Modelling Files

## Scratch in ROS and RViz
### Explanation of URDF Files and Launch ROS Files
1. Install and use RVIZ and Other Packages
    * "sudo apt-get install ros-melodic-rviz"
    * "roscore"
    * Go to new terminal "rosrun rviz rviz"
2. Install for URDF modeling
    * "sudo apt-get install ros-melodic-urdf"
    * "sudo apt-get install liburdfdom-tools"
3. Create the Workspace and Catkin Package
    * "mkdir -p ~/robot_ws/src"
    * "cd ~/robot_ws"
    * "catkin_make"
    * "source ~/robot_ws/devel/setup.bash"
    * "$ROS_PACKAGE_PATH"
    * "cd ~/robot_ws/src"
    * "catkin_create_pkg robot_model_pkg roscpp tf2 geometry_msgs urdf rviz joint_state_publisher_gui"
        * Dependencies:
        - roscpp: cpp and ROS.
        - tf2: Lets the user keep track of multiple coordinate frames over time.
        - geometry_msgs: Messages for common geometric primitives such as points, vectors, and poses.
        - urdf: C++ parser for URDF package
        - rviz: RVIZ
        - joint_state_publisher_gui: Contains a GUI tool for setting and publishing joint state values for a given URDE
4. Create the URDF file and launc file.
    