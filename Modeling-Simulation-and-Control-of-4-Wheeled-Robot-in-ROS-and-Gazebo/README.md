# Modeling, Simulation and Control of 4 Wheeled Robot in ROS and Gazebo

## XML Files
## robot.xacro:
    <?xml version="1.0"?>
    <!-- ##################################### -->
    <!-- DESCRIPTION OF THE FOUR WHEELED ROBOT -->
    <!-- Made by Alexsandar Haber - October 23 -->
    <!-- ##################################### -->

    <robot name="differential_drive_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
        <!-- Body Dimensions -->
        <xacro:property name="body_link_x_dim" value="1"/>
        <xacro:property name="body_link_y_dim" value="0.6"/>
        <xacro:property name="body_link_z_dim" value="0.3"/>
        
        <!-- Wheel Dimensions -->
        <xacro:property name="wheel_link_radius" value="0.15"/>
        <xacro:property name="wheel_link_length" value="0.1"/>
        <xacro:property name="wheel_link_z_location" value="-0.1"/>

        <!-- Material Density -->
        <xacro:property name="body_density" value="2710.0"/>
        <xacro:property name="wheel_density" value="2710.0"/>

        <!-- Pi constant -->
        <xacro:property name="pi_const" value="3.14159265"/>
        
        <!-- Robot body and wheel mass -->
        <xacro:property name="body_mass" value="${body_density*body_link_x_dim*body_link_y_dim*body_link_z_dim}"/>
        <xacro:property name="wheel_mass" value="${wheel_density*pi_const*wheel_link_radius*wheel_link_radius*wheel_link_length}"/>
        
        <!-- Moments of inertia of the wheel -->
        <xacro:property name="Iz_wheel" value="${0.5*wheel_mass*wheel_link_radius*wheel_link_radius}"/>
        <xacro:property name="I_wheel" value="${(1.0/12.0)*wheel_mass*(3.0*wheel_link_radius/wheel_link_radius+wheel_link_length*wheel_link_length)}"/>
        
        <!-- This macro defiens the complete inertial seciton of the wheel -->
        <!-- It is used later in cod -->
        <xacro:macro name="inertia_wheel">
            <inertial>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <mass value="${wheel_mass}"/>
                <inertia ixx="${I_wheel}" ixy="0.0" ixz="0.0" iyy="${I_wheel}" iyz="0.0" izz="${Iz_wheel}"/>
            </inertial>
        </xacro:macro>

        <!-- Over here we include the file that dfefines extra Gazebo options and motion control driver -->
        <xacro:include filename="$(find robot_model_pkg)/urdf/robot.gazebo"/>
        
        <!-- ############################## -->
        <!-- Now we define Links and Joints -->
        <!-- ############################## -->

        <!-- In ROS, created objects calls by links. For example wheel1 and body. They are two different 3D shapes but we call them as links. So in this case, joints are coming meaning that connecitons between two different shapes. We won't call these conencitons as links -->

        <!-- We need to have this link, otherwise Gazebo will complain -->
        <link name="dummy"></link>
        <joint name="dummy_joint" type="fixed">
            <parent link="dummy"/>
            <child link="body_link"/>
        </joint>

        <!-- ######################################## -->
        <!-- We will start from the body of our robot -->
        <!-- ######################################## -->
        <link name="body_link">
            <visual>
                <geometry>
                    <box size="${body_link_x_dim} ${body_link_y_dim} ${body_link_z_dim}"/>
                </geometry>
            </visual>

            <collision>
                <geometry>
                    <box size="${body_link_x_dim} ${body_link_y_dim} ${body_link_z_dim}"/>
                </geometry>
                <origin xyz="0 0 0" rpy="0 0 0"/>
            </collision>

            <inertial>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <mass value="${body_mass}"/>

                <inertia ixx="${(1/12)*body_mass*(body_link_y_dim*body_link_y_dim+body_link_z_dim*body_link_z_dim)}" ixy="0" ixz="0" iyy="${(1/12)*body_mass*(body_link_x_dim*body_link_x_dim+body_link_z_dim*body_link_z_dim)}" iyz="0" izz="${(1/12)*body_mass*(body_link_y_dim*body_link_y_dim+body_link_x_dim*body_link_x_dim)}"/>
            </inertial>
        </link>

        <!-- ########################################################### -->
        <!-- Now we start to back right wheel of the robot and the joint -->
        <!-- ########################################################### -->
        <joint name="wheel1_joint" type="continuous">
            <parent link="body_link"/>
            <child link="wheel1_link"/>
            <origin xyz="${-body_link_x_dim/2+1.2*wheel_link_radius} ${-body_link_y_dim/2-wheel_link_length/2} ${wheel_link_z_location}" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
            <limit effort="1000" velocity="1000"/>
            <dynamics damping="1.0" friction="1.0"/>
        </joint>

        <link name="wheel1_link">
            <visual>
                <origin xyz="0 0 0" rpy="1.570795 0 0"/>
                <geometry>
                    <cylinder radius="${wheel_link_radius}" length="${wheel_link_length}"/>
                </geometry>
            </visual>

            <collision>
                <origin xyz="0 0 0" rpy="1.570795 0 0"/>
                <geometry>
                    <cylinder radius="${wheel_link_radius}" length="${wheel_link_length}"/>
                </geometry>
            </collision>

            <!-- We defined this macro above and we will call it in each wheel link -->
            <xacro:inertia_wheel/>
        </link>

        <!-- ########################################################## -->
        <!-- Now we start to back left wheel of the robot and the joint -->
        <!-- ########################################################## -->
        <joint name="wheel2_joint" type="continuous">
            <parent link="body_link"/>
            <child link="wheel2_link"/>
            <origin xyz="${-body_link_x_dim/2+1.2*wheel_link_radius} ${body_link_y_dim/2+wheel_link_length/2} ${wheel_link_z_location}" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
            <limit effort="1000" velocity="1000"/>
            <dynamics damping="1.0" friction="1.0"/>
        </joint>

        <link name="wheel2_link">
            <visual>
                <origin xyz="0 0 0" rpy="1.570795 0 0"/>
                <geometry>
                    <cylinder radius="${wheel_link_radius}" length="${wheel_link_length}"/>
                </geometry>
            </visual>

            <collision>
                <origin xyz="0 0 0" rpy="1.570795 0 0"/>
                <geometry>
                    <cylinder radius="${wheel_link_radius}" length="${wheel_link_length}"/>
                </geometry>
            </collision>

            <!-- We defined this macro above and we will call it in each wheel link -->
            <xacro:inertia_wheel/>
        </link>
        
        <!-- ############################################################ -->
        <!-- Now we start to front right wheel of the robot and the joint -->
        <!-- ############################################################ -->
        <joint name="wheel3_joint" type="continuous">
            <parent link="body_link"/>
            <child link="wheel3_link"/>
            <origin xyz="${body_link_x_dim/2-1.2*wheel_link_radius} ${-body_link_y_dim/2-wheel_link_length/2} ${wheel_link_z_location}" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
            <limit effort="1000" velocity="1000"/>
            <dynamics damping="1.0" friction="1.0"/>
        </joint>

        <link name="wheel3_link">
            <visual>
                <origin xyz="0 0 0" rpy="1.570795 0 0"/>
                <geometry>
                    <cylinder radius="${wheel_link_radius}" length="${wheel_link_length}"/>
                </geometry>
            </visual>

            <collision>
                <origin xyz="0 0 0" rpy="1.570795 0 0"/>
                <geometry>
                    <cylinder radius="${wheel_link_radius}" length="${wheel_link_length}"/>
                </geometry>
            </collision>

            <!-- We defined this macro above and we will call it in each wheel link -->
            <xacro:inertia_wheel/>
        </link>

        <!-- ########################################################### -->
        <!-- Now we start to front left wheel of the robot and the joint -->
        <!-- ########################################################### -->
        <joint name="wheel4_joint" type="continuous">
            <parent link="body_link"/>
            <child link="wheel4_link"/>
            <origin xyz="${body_link_x_dim/2-1.2*wheel_link_radius} ${body_link_y_dim/2+wheel_link_length/2} ${wheel_link_z_location}" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
            <limit effort="1000" velocity="1000"/>
            <dynamics damping="1.0" friction="1.0"/>
        </joint>

        <link name="wheel4_link">
            <visual>
                <origin xyz="0 0 0" rpy="1.570795 0 0"/>
                <geometry>
                    <cylinder radius="${wheel_link_radius}" length="${wheel_link_length}"/>
                </geometry>
            </visual>

            <collision>
                <origin xyz="0 0 0" rpy="1.570795 0 0"/>
                <geometry>
                    <cylinder radius="${wheel_link_radius}" length="${wheel_link_length}"/>
                </geometry>
            </collision>

            <!-- We defined this macro above and we will call it in each wheel link -->
            <xacro:inertia_wheel/>
        </link>
    </robot>

## robot.gazebo
    <?xml version="1.0"?>
    <!-- ####################################################### -->
    <!-- GAZEBO ADDITIONAL DESCRIPTION OF THE FOUR WHEELED ROBOT -->
    <!-- Made by Alexsandar Haber          -          October 23 -->
    <!-- ####################################################### -->

    <robot>
        <!-- Everyting described here: http://classic.gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros -->

        <!-- mu1 and mu2 are friction coefficients -->
        <gazebo reference="body_link">
        <mu1>0.2</mu1>
        <mu2>0.2</mu2>
        <material>Gazebo/Red</material>
        </gazebo>
        
        <gazebo reference="wheel1_link">
        <mu1>0.2</mu1>
        <mu2>0.2</mu2>
        <material>Gazebo/Yellow</material>
        </gazebo>

        <gazebo reference="wheel2_link">
        <mu1>0.2</mu1>
        <mu2>0.2</mu2>
        <material>Gazebo/Yellow</material>
        </gazebo>
        
        <gazebo reference="wheel3_link">
        <mu1>0.2</mu1>
        <mu2>0.2</mu2>
        <material>Gazebo/Yellow</material>
        </gazebo>

        <gazebo reference="wheel4_link">
        <mu1>0.2</mu1>
        <mu2>0.2</mu2>
        <material>Gazebo/Yellow</material>
        </gazebo>
        
        <!-- Controller for the four wheeled robot -->
        <gazebo>
            <plugin name="skid_steer_drive_controller" filename="libgazebo_ros_skid_steer_drive.so">

            <!-- Control update raze in Hz -->
            <updateRate>100.0</updateRate>        
            
            <!-- Leave this empty otherwise there will be problems with sending control commands -->
            <robotNamespace></robotNamespace>
            
            <!-- Robot Kinematics -->
            <leftFrontJoint>wheel4_joint</leftFrontJoint>
            <rightFrontJoint>wheel3_joint</rightFrontJoint>
            <leftRearJoint>wheel2_joint</leftRearJoint>
            <rightRearJoint>wheel1_joint</rightRearJoint>
            <wheelSeparation>${-body_link_y_dim+wheel_link_length}</wheelSeparation>
            <wheelDiameter>${wheel_link_radius}</wheelDiameter>
            
            <!-- Maximum torque which the wheels can produce, in Nm, defaults to 5 Nm -->
            <torque>1000</torque>
            
            <!-- Topic to receive geometry_msgs/Twist message commands, default to "cmd_vel" -->
            <commandTopic>cmd_vel</commandTopic>
            
            <!-- Topic to publish nac_msgs/Odometry messages, defaults to "odom" -->
            <odometryTopic>odom</odometryTopic>
            
            <!-- Odometry frame, defaults to "odom" -->
            <odometryTopic>odom</odometryTopic>
            
            <!-- Robot frame to calculate odometry from, defaults to "base_footprint" -->
            <robotBaseFrame>dummy</robotBaseFrame>

            <!-- Set to true to publish transforms for the wheel links, defaults to false -->
            <publishWheelTF>true</publishWheelTF>

            <!-- Set to true to publish transforms for the wheel links, defaults to true -->
            <publishOdom>true</publishOdom>

            <!-- Set to true to publish sensor_msgs/JointState on /joint_states for the wheel joints, defaults to false -->
            <publishWheelJointStates>true</publishWheelJointStates>

            <!-- This part required by Gazebo -->
            <covariance_x>0.0001</covariance_x>
            <covariance_y>0.0001</covariance_y>
            <covariance_yaw>0.01</covariance_yaw>

            </plugin>
        </gazebo>
    </robot>

## robot_xacro.launch
    <?xml version="1.0"?>
    <!-- ############################################################### -->
    <!-- LAUNCH FİLE FOR THE GAZEBO SIMULATION OF THE FOUR WHEELED ROBOT -->
    <!-- Made by Alexsandar Haber              -              October 23 -->
    <!-- ############################################################### -->
    <launch>
        <include file="$(find gazebo_ros)/launch/empty_world.launch">
            <arg name="paused" value="false"/>
            <arg name="use_sim_time" value="true"/>
            <arg name="gui" value="true"/>
            <arg name="headless" value="false"/>
            <arg name="debug" value="false"/>
        </include>
        
        <!-- Load the robot description -->
        <param name="robot_description" command="$(find xacro)/xacro '$(find robot_model_pkg)/urdf/robot.xacro'"/>
        
        <!-- Robot state publisher node -->
        <node name="robot_description" pkg="robot_state_publisher" type="robot_state_publisher"/>
        
        <!-- Spawn the model -->
        <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model"  output="screen" respawn="false" args="-urdf -model robot_model -param robot_description"/>
    </launch>

