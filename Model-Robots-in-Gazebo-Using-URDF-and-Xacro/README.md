# Explanation of URDF Files and Launch ROS Files

## XML Files
## robot.xacro:
    <?xml version="1.0"?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_robot">

    <!-- Here we define the model parameters -->

    <!-- Geometry -->

    <xacro:property name="base_link_length" value="05"/>
    <xacro:property name="base_link_radius" value="0.2"/>

    <xacro:property name="first_link_length" value="0.5"/>
    <xacro:property name="first_link_radius" value="04"/>

    <xacro:property name="second_link_length" value="1.0"/>
    <xacro:property name="second_link_height" value="0.1"/>
    <xacro:property name="second_link_width" value="0.1"/>

    <!-- Joint Limits -->

    <xacro:property name="limits_revolute_upper" value="3.14"/>
    <xacro:property name="limits_revolute_lower" value="-3.14"/>
    <xacro:property name="limits_revolute_velocity" value="3.14"/>
    <xacro:property name="limits_revolute_effort" value="300"/>

    <xacro:property name="limits_prismatic_upper" value="0.5"/>
    <xacro:property name="limits_prismatic_lower" value="-0.5"/>
    <xacro:property name="limits_prismatic_velocity" value="0.8"/>
    <xacro:property name="limits_prismatic_effort" value="300"/>

    <xacro:include filename="$(find robot_model3_pkg)/urdf/robot.gazebo"/>

        <link name="base_link">
            <visual>
                <geometry>
                    <cylinder radius="${base_link_radius}" length="${base_link_length}"/>
                    
                    <!-- origin - the reference frame of the visual element with respect to the reference frame of the link -->
                    <!-- rpy - r: roll, p: pitch and y: yaw -->
                    <!-- xyz - x, y, z offsets -->

                    <origin xyz="0 0 0" rpy="0 0 0"/>
                </geometry>
            </visual>
            <collision>
                <geometry>
                    <cylinder radius="${base_link_radius}" length="${base_link_length}"/>
                </geometry>
                <origin xyz="0 0 0" rpy="0 0 0"/>
            </collision>
            <inertial>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <mass value="1"/>
                <inertia ixx="100" ixy="0" ixz="0" iyy="100" iyz="0" izz="100"/>
            </inertial>
        </link>

        <joint name="joint_1" type="revolute">
            <parent link="base_link"/>
            <child link="first_link"/>
            <origin xyz="0 0 ${base_link_length/2}"/>
            <axis xyz="0 0 1.0"/>
            <limit lower="${limits_prismatic_lower}" upper="${limits_prismatic_upper}" effort="${limits_prismatic_effort}" velocity="${limits_prismatic_velocity}"/>
        </joint>

        <link name="first_link">
            <visual>
                <geometry>
                    <cylinder radius="${first_link_radius}" length="${first_link_length}"/>
                </geometry>
                <origin xyz="0 0 ${first_link_length/2}" rpy="0 0 0"/>
            </visual>

            <collision>
                <geometry>
                    <cylinder radius="${first_link_radius}" length="${first_link_length}"/>
                </geometry>
                <origin xyz="0 0 ${first_link_length/2}" rpy="0 0 0"/>
            </collision>

            <inertial>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <mass value="0"/>
                <inertia ixx="100" ixy="0" ixz="0" iyy="100" iyz="0" izz="100"/>
            </inertial>
        </link>

        <joint name="joint_2" type="prismatic">
            <parent link="first_link"/>
            <child link="second_link"/>
            <origin xyz="0 0 ${first_link_length}"/>
            <axis xyz="0 1.0 0"/>
            <limit lower="${limits_prismatic_lower}" upper="${limits_prismatic_upper}" effort="${limits_prismatic_effort}" velocity="${limits_prismatic_velocity}"/>
        </joint>

        <link name="second_link">
            <visual>
                <geometry>
                    <box size="${second_link_width} ${second_link_length} ${second_link_height}"/>
                </geometry>
                <origin xyz="0 0 ${second_link_height/2}" rpy="0 0 0"/>
            </visual>

            <collision>
                <geometry>
                    <box size="${second_link_width} ${second_link_length} ${second_link_height}"/>
                </geometry>
                <origin xyz="0 0 ${second_link_height/2}" rpy="0 0 0"/>
            </collision>

            <inertial>
                <origin xyz="0 0 ${second_link_height/2}" rpy="0 0 0"/>
                <mass value="1"/>
                <inertia ixx="100" ixy="0" ixz="0" iyy="100" iyz="0" izz="100"/>
            </inertial>
        </link>
    </robot>

## robot.gazebo
    <?xml version="1.0"?>
    <robot>
    <gazebo>
        <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
            <robotNamespace>/simple_robot</robotNamespace>
        </plugin>
    </gazebo>

    <gazebo reference="base_link">
    <mul1>0.2</mul1>
    <mul2>0.2</mul2>
    <material>Gazebo/Green</material>
    </gazebo>

    <gazebo reference="first_link">
    <mul1>0.2</mul1>
    <mul2>0.2</mul2>
    <material>Gazebo/Red</material>
    </gazebo>

    <gazebo reference="second_link">
    <mul1>0.2</mul1>
    <mul2>0.2</mul2>
    <material>Gazebo/Blue</material>
    </gazebo>
    </robot>
## robot_xacro.launch
    <?xml version="1.0"?>
    <launch>
        <include file="${find gazebo_ros}/launch/empty_world.launch"/>
            <arg name="paused" value="false"/>
            <arg name="use_sim_time" value="true"/>
            <arg name="gui" value="true"/>
            <arg name="headless" value="false"/>
            <arg name="debug" value="false"/>
        </include>

        <rosparam name="robot_description" command="${find xacro}/xacro '${find robot_model_pkg}/urdf/robot.xacro'"/>
        <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"  output="screen" respawn="false" args="-urdf -model robot22 -param robot_description"/>
    </launch>