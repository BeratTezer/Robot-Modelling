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
    * "echo $ROS_PACKAGE_PATH"
    * "cd ~/robot_ws/src"
    * "catkin_create_pkg robot_model_pkg roscpp tf2 geometry_msgs urdf rviz joint_state_publisher_gui"
        * Dependencies:
        - roscpp: cpp and ROS.
        - tf2: Lets the user keep track of multiple coordinate frames over time.
        - geometry_msgs: Messages for common geometric primitives such as points, vectors, and poses.
        - urdf: C++ parser for URDF package
        - rviz: RVIZ
        - joint_state_publisher_gui: Contains a GUI tool for setting and publishing joint state values for a given URDE
4. Create the URDF file and launch file.
    - "cd ~/robot_ws/src/robot_model_pkg"
    - "mkdir urdf"
    - "mkdir launch"
    - "cd urdf"
    - "gedit robot.urdf" You can find it in README.md file.
    - "cd .."
    - "cd launch"
    - "gedit robot.launch" You can find it in README.md file.
    - "cd ~/robot_ws/"
    - "catkin_make"
        * If your "joint-state-publisher-gui" couldn't found, you can solve this problem with "sudo apt install ros-melodic-joint-state-publisher-gui"
5. Test.
    - Go to new terminal and write "roscore"
    - Then return the terminal you changed adn write "roslaunch robot_model_pkg robot.launch".
    - Now you will see RViz screen. Click "add" and select "robot model".
    - Change the fixed frames name from "map" to "base_link"
    - Then click "Save Configg as", go to "robot_model_pkg" and name it "robot.rviz"
    - "cd robot_model_pkg/"
    - "cd urdf/"
    - "gedit robot.urdf" You can find this file inside of the README.md
    - "roslaunch robot_model_pkg robot.launch"
    - Now you are able to control the green box from small GUI.

### Xacro and URDF to Parametrize and Model Robots in RViz
1. Go to a clean termianl and write "sudo apt-get install ros-melodic-rviz"
2. Go to new terminal and start ros by "roscore"
3. Go back to first terminal and run "rosrun rviz rviz"
4. Installations.
    - "sudo apt-get install ros-melodic-urdf"
    - "sudo apt-get install ros-melodic-xacro"
    - "sudo apt-get install liburdfdom-tools"
5. Create a workspace and catkin package
    - "mkdir -p ~/robot2_ws/src"
    - "cd ~/robot2_ws"
    - "catkin_make"
    - "source ~/robot2_ws/devel/setup.bash"
    - "echo $ROS_PACKAGE_PATH"
    - "cd ~/robot2_ws/src"
    - "catkin_create_pkg robot2_model_pkg roscpp tf2 geometry_msgs urdf rviz joint_state_publisher_gui xacro"
        - Dependencies
        * roscpp: Cpp and ROS
        * tf2: Lets the user keep track of multiple coordinate frames over time
        * geometry_msgs: Messages for common geometric primitives such as points, vectors, and poses.
        * urdf: C++ parser for URDF package
        * rviz: RVIZ
        * joint_state_publisher_gui: 
        * xacro: XACRO
6. Create the XACRO and Launch files.
    - "cd ~/robot2_ws/src/robot2_model_pkg/"
    - "mkdir urdf"
    - "mkdir launch"
    - "cd urdf"
    - "gedit material_color.xacro". You can find the content in README.md
    - "gedit robot.xacro". You can find the content in README.md
    ![Untitled](https://github.com/BeratTezer/Robot-Modelling/assets/64587561/9eea0efe-ea65-4550-929c-f0d0f346f3c1)
    - "cd ~/robot2_ws/src/robot2_model_pkg/launch"
    - "gedit robot_xacro.launch" You can find the content in README.md
    - Return the original workspace "cd ~/robot2_ws"
    - "catkin_make"
    - "roslaunch robot2_model_pkg robot_xacro.launch" (If you close the roscore we ran in step 2, go a new terminal an start before this line)
        * Add->Robot Model
        * Change the fixed frame name from "map" to "base_link"
        * Add "TF", now you can see objects names.

### Model Robots in Gazebo From Scratch by Using URDF and Xacro
1. Go to a terminal and write "gazebo". If not, you have to install Gazebo by properly installing ROS.
    - If you get "[Err] [REST.cc:205] Error in REST request", "gedit ~/.ignition/fuel/config.yaml" and change the url from "url: https://api.ignitionfuel.org" to this "url: https://api.ignitionrobotics.org"
2. Necessary Packages
    - "sudo apt-get install ros-melodic-gazebo-ros-pkgs"
    - "sudo apt-get install ros-melodic-gazebo-msgs"
    - "sudo apt-get install ros-melodic-gazebo-plugins"
    - "sudo apt-get install ros-melodic-gazebo-ros-control"
3. Create a workspace and catkin package
    - "mkdir -p ~/robot3_ws/src"
    - "cd ~/robot3_ws"
    - "catkin_make"
    - "source ~/robot3_ws/devel/setup.bash"
    - "echo $ROS_PACKAGE_PATH"
    - "cd ~/robot3_ws/src"
    - "catkin_create_pkg robot_model3_pkg gazebo_msgs gazebo_plugins gazebo_ros gazebo_ros_control mastering_ros_robot_description_pkg"
        * Dependencies:
            - gazebo_msgs
            - gazebo_plugins
            - gazebo_ros 
            - gazebo_ros_control 
            - mastering_ros_robot_description_pkg
4. Create the source files defining the robot geometry
    - "cd ~/robot3_ws/src/robot_model3_pkg"
    - "mkdir urdf"
    - "cd ~/robot3_ws/src/robot_model3_pkg/urdf"
    - "gedit robot.xacro" You can file the content in README.md
    - "gedit robot.gazebo" You can file the content in README.md
    - "cd .."
    - "mkdir launch"
    - "cd ~/robot3_ws/src/robot_model3_pkg/launch"
    - "gedit robot_xacro.launch" You can file the content in README.md
    ![Untitled](https://github.com/BeratTezer/Robot-Modelling/assets/64587561/be97e5c6-c62f-4b6f-8ee4-2bb5733aafe6)
5. Launch the model in Gazebo
    - "roslaunch robot_model_pkg robot_xacro.launch"

### TurtleBot3 Robot in ROS and Simulate Movement in Gazebo
1. Create the folder and clone the necessary TurtleBot3 files
    - "mkdir -p ~/test_turtlebot/src"
    - "cd ~/test_turtlebot/src"
    - "git clone https://github.com/ROBOTIS-GIT/turtlebot3.git"
    - "git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git"
    - "git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git"
    - "cd ~/test_turtlebot"
2. Create the workspace and specif the source
    - "catkin_make"
    - "source ~/test_turtlebot/devel/setup.bash"
3. Simulate the robot motion in Gazebo
    - We have three robot opitons. Burger, Waffle, and Waffle Pi. Now we will select the "burger"
    - "export TURTLEBOT3_MODEL=burger"
4. Launch the world
    - Empty World: "roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch"
    - Another World: "roslaunch turtlebot3_gazebo turtlebot3_world.launch"
    - Teleoperation Node:
        * Go to the new terminal
        * Specify the source "source ~/test_turtlebot/devel/setup.bash"
        * Specify the model "export TURTLEBOT3_MODEL=burger"
        * Launch the teleoperation node "roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch". Now w are able to control the robot.
5. Final test
    - Go to new terminal "cd ~/test_turtlebot"
    - Specify the source "source ~/test_turtlebot/devel/setup.bash"
    - Specify the model "export TURTLEBOT3_MODEL=burger"
    - Launch the house "roslaunch turtlebot3_gazebo turtlebot3_house.launch"
    - Go to new terminal "cd ~/test_turtlebot"
    - Specify the source "source ~/test_turtlebot/devel/setup.bash"
    - Specify the model "export TURTLEBOT3_MODEL=burger"
    - Launch the teleoperation node "roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch". Now we are able to control the robot again.

### Create URDF File and Model of Four Wheeled Mobile Robot in ROS
1. Install RVIZ nececities
    - "sudo apt-get install ros-melodic-rviz"
    - "roscore"
    - "rosrun rviz rviz", if works continue.
2. Install for URDF Modeling
    - "sudo apt-get install ros-melodic-urdf"
    - "sudo apt-get install liburdfdom-tools"
    - "sudo apt-get install ros-melodic-xacro"
3. Create the workspace and catkin package
    - "mkdir -p ~/robot_dd/src"
    - "cd ~/robot_dd"
    - "catkin make"
    - "source ~/robot_dd/devel/setup.bash"
    - "echo $ROS_PACKAGE_PATH"
    - "cd ~/robot_dd/src"
    - Create the packages "catkin_create_pkg robot_dd_model_pkg roscpp tf2 geometry_msgs urdf rviz joint_state_publisher_gui"
        * Dependencies:
        * roscpp: CPP and ROS
        * tf2: Lets the user keep track of multiple frames over time
        * geometry_msgs: Messages for common geometric primitives such as points, vectors, and poses
        * urdf: C++ parser for UDF package
        * rviz: RVIZ
        * joint_state_publisher_gui: Contains a GUI tool for setting and publishing joint state values for a given URDF
4. Create the URDF file and Launch file
    - "cd ~/robot_dd/src/robot_dd_model_pkg"
    - "mkdir urdf"
    - "mkdir launch"
    - "cd urdf"
    - "gedit robot.urdf" You can reach the code on README
    - "cd ~/robot_dd/src/robot_dd_model_pkg/launch"
    - "gedit robot.launch" You can reach the code on README
    - "cd ~/robot_dd"
    - "catkin_make" again
    ![Untitled](https://github.com/BeratTezer/Robot-Modelling/assets/64587561/ab7c2186-fc80-49e9-9fd3-3e29dcc42ef0)
5. Test
    - Open a new terminal
    - "roscore"
    - In original terminal, write "roslaunch robot_dd_model_pkg robot.launch"
    - Change the fixed frame name from "map" to "body_link" then add "RobotModel".
    - Too see the move you give from joint_state_publisher_gui, click add, then select the Axes make Axes' reference wheel any of them and try to change wheel1_joint value. You will see the difference in simulation.

### Modeling, Simulation and Control of 4 Wheeled Robot in ROS and Gazebo
1. Install necessary packages
    - Open a terminal and write "gazebo". If Gazebo don't open, install it properly by installing ROS.
    - Download these:
        * "sudo apt-get install ros-melodic-gazebo-ros-pkgs"
        - Provide the necessary interfaces to simulate a robot in Gazebo 3D rigid body simulator for robots. It integrates with ROS using ROS messages, services and dynamic reconfigure.
        * "sudo apt-get install ros-melodic-gazebo-msgs"
        - Messages and service data structures for interfacing ROS and Gazebo.
        * "sudo apt-get install ros-melodic-gazebo-plugins"
        - Robot independent Gazebo plugins for sensors, motor and dynamic reconfigurable components.
        * "sudo apt-get install ros-melodic-gazebo-ros-control"
        - Controllers to communicate between ROS and Gazebo.
        * "sudo apt-get install ros-melodic-teleop-twist-keboard"
        - Package for controlling the robot by using keyboard keys.
2. Create the Workspace and Catkin package.
    - "mkdir -p ~/robot_four/src"
    - "cd robot_four"
    - "catkin_make"
    - "source ~/robot_four/devel/setup.bash"
    - "echo $ROS_PACKAGE_PATH"
    - "cd ~/robot_four/src"
    - "catkin_create_pkg robot_model_pkg gazebo_msgs gazebo_plugins gazebo_ros gazebo_ros_control"
        * Dependencies:
        * "gazebo_msgs"
        * "gazebo_plugins"
        * "gazebo_ros"
        * "gazebo_ros_control"
3. Create the source files defining robot geometry
    - "cd ~/robot_four/src/robot_model_pkg"
    - "mkdir urdf"
    - "cd ~/robot_four/src/robot_model_pkg/urdf"
    - "gedit robot.xacro" You can find the content in related README file.
    - ![Untitled](https://github.com/BeratTezer/Robot-Modelling/assets/64587561/ff0ae5f7-4b33-4aa0-b0ff-5bdfbde2757e)

    - ""
