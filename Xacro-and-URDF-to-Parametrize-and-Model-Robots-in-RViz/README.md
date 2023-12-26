# Explanation of URDF Files and Launch ROS Files

## XML Files
## robot.xacro:
    <?xml version="1.0"?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_robot">

    <!-- Here we define the model parameters -->

    <!-- Geometry -->

    <xacro:property name="base_link_length" value="0.05"/>
    <xacro:property name="base_link_radius" value="0.2"/>

    <xacro:property name="first_link_length" value="0.5"/>
    <xacro:property name="first_link_radius" value="0.04"/>

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

    <!-- Load material properties, in this case colors -->
    <xacro:include filename="$(find robot2_model_pkg)/urdf/material_color.xacro"/>
        <link name="base_link">
            <visual>
                <geometry>
                    <cylinder length="${base_link_length}" radius="${base_link_radius}" />
                </geometry>
                <!-- Origin - the reference frame of the visual element with respect to the reference frame of the link -->
                <!-- rpy - r is roll, p-pitch and y -yaw -->
                <!-- xyz - x,y,z -offsets -->
                <origin rpy="0 0 0" xyz="0 0 0" />
                <material name="blue" />
            </visual>
        </link>
        <joint name="joint_1" type="revolute">
            <parent link="base_link"/>
            <child link="first_link"/>
                <origin xyz="0 0 ${base_link_length/2}"/>
                <axis xyz="0 0 1"/>
                <limit effort="${limits_revolute_effort}" velocity="${limits_revolute_velocity}" lower="${limits_revolute_lower}" upper="${limits_revolute_upper}"/>
        </joint>

        <link name="first_link">
        <visual>
            <geometry>
                <cylinder length="${first_link_length}" radius="${first_link_radius}"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 ${first_link_length/2}"/>
            <material name="red"/>
        </visual>
        </link>

        <joint name="joint_2" type="prismatic">
            <parent link="first_link"/>
            <child link="second_link"/>
                <origin xyz="0 0 ${first_link_length}"/>
                <axis xyz="0 1 0"/>
                <limit effort="${limits_prismatic_effort}" velocity="${limits_prismatic_velocity}" lower="${limits_prismatic_lower}" upper="${limits_prismatic_upper}"/>

        </joint>

        <link name="second_link">
        <visual>
            <geometry>
                <box size="${second_link_width} ${second_link_length} ${second_link_height}"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 ${second_link_height/2}"/>
            <material name="green"/>
        </visual>
        </link>
    </robot>

## material_color.xacro:
    <?xml version="1.0"?>
    <robot>
        <material name="black">
            <color rgba="0.0 0.0 0.0 1.0"/>
        </material>
        <material name="blue">
            <color rgba="0.0 0.0 0.9 1.0"/>
        </material>
        <material name="green">
            <color rgba="0.0 0.9 0.0 1.0"/>
        </material>
        <material name="red">
            <color rgba="0.9 0.0 0.0 1.0"/>
        </material>
        <material name="white">
            <color rgba="1.0 1.0 1.0 1.0"/>
        </material>
    </robot>

## robot_xacro.launch:
    <?xml version="1.0"?>
    <launch>
        <param name="robot_description" command="$(find xacro)/xacro '$(find robot2_model_pkg)/urdf/robot.xacro'"/>

        <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" />
        <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
        <node name="rviz" pkg="rviz" type="rviz" args="-d $(find robot2_model_pkg)/robot.rviz" required="true"/>
    </launch>
