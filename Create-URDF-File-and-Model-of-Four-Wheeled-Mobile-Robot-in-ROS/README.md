# Explanation of URDF Files and Launch ROS Files

## XML Files
## robot.urdf:
    <?xml version="1.0"?>
    <launch>
        <arg name="model"/>
        <param name="robot_description" textfile="$(find robot_dd_model_pkg)/urdf/robot.urdf"/>
        <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui"/>
        <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
        <node name="rviz" pkg="rviz" type="rviz" args="-d $(find robot_dd_model_pkg)/robot.rviz" required="true"/>
    </launch>
## robot.launch:
    <?xml version="1.0"?>
    <!-- http://wiki.ros.org/urdf/XML/link -->
    <robot name="fourwheel_robot">
        <!-- This is the body of the robot -->
        <link name="body_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="1 0.6 0.3"/>
                </geometry>
                <material name="red">
                    <color rgba="1 0 0 1"/>
                </material>
            </visual>
        </link>
        <!-- This is the back right wheel of the robot -->
        <joint name="wheel1_joint" type="continuous">
            <parent link="body_link"/>
            <child link="wheel1_link"/>
            <origin xyz="-0.3 -0.35 -0.1" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
        </joint>
        <link name="wheel1_link">
            <visual>
                <origin xyz="0 0 0" rpy="1.570795 0 0"/>
                <geometry>
                    <cylinder radius="0.15" length="0.1"/>
                </geometry>
                <material name="yellow">
                    <color rgba="1 1 0 1"/>
                </material>
            </visual>
        </link>
        <!-- This is the back left wheel of the robot -->
        <joint name="wheel2_joint" type="continuous">
            <parent link="body_link"/>
            <child link="wheel2_link"/>
            <origin xyz="-0.3 0.35 -0.1" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
        </joint>
        <link name="wheel2_link">
            <visual>
                <origin xyz="0 0 0" rpy="1.570795 0 0"/>
                <geometry>
                    <cylinder radius="0.15" length="0.1"/>
                </geometry>
                <material name="yellow">
                    <color rgba="1 1 0 1"/>
                </material>
            </visual>
        </link>
        <!-- This is the front right wheel of the robot -->
        <joint name="wheel3_joint" type="continuous">
            <parent link="body_link"/>
            <child link="wheel3_link"/>
            <origin xyz="0.3 -0.35 -0.1" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
        </joint>
        <link name="wheel3_link">
            <visual>
                <origin xyz="0 0 0" rpy="1.570795 0 0"/>
                <geometry>
                    <cylinder radius="0.15" length="0.1"/>
                </geometry>
                <material name="yellow">
                    <color rgba="1 1 0 1"/>
                </material>
            </visual>
        </link>
        <!-- This is the front left wheel of the robot -->
        <joint name="wheel4_joint" type="continuous">
            <parent link="body_link"/>
            <child link="wheel4_link"/>
            <origin xyz="0.3 0.35 -0.1" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
        </joint>
        <link name="wheel4_link">
            <visual>
                <origin xyz="0 0 0" rpy="1.570795 0 0"/>
                <geometry>
                    <cylinder radius="0.15" length="0.1"/>
                </geometry>
                <material name="yellow">
                    <color rgba="1 1 0 1"/>
                </material>
            </visual>
        </link>
    </robot>