# Explanation of URDF Files and Launch ROS Files

## XML Files
## robot.urdf:
    <?xml version="1.0"?>
    <!-- http://wiki.ros.org/urdf/XML/link -->
    <robot name="simple_robot">
        <link name="base_link">
            <visual>
                <geometry>
                    <cylinder length="0.05" radius="0.2"/>
                </geometry>
                <!--origin - the reference frame of the visual element with respect to the reference frame of the link -->
                <!-- rpy - r is roll, p-pitch and y -yaw -->
                <!-- xyz - x,y,z -offsets -->
                <origin rpy = "0 0 0" xyz="0 0 0"/>
                <material name="yellow">
                    <color rgba="1 1 0 1"/>
                </material>
            </visual>
        </link>
    </robot>

## robot.launch:
    <?xml version="1.0" ?>
    <launch>
        <arg name="model" />
        <param name="robot_description" textfile="$(find robot_model_pkg)/urdf/robot.urdf" />
        <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" tpye="joint_state_publisher_gui" />
        <node name="robot_state_publisher" pkg="robot_state_publisher" tpye="robot_state_publisher" />
        <node name="rviz" pkg="rviz" tpye="rviz" args="-d $(find robot_model_pkg)/robot.rviz" required="true"/>
    </launch>

