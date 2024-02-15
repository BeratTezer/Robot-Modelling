# Contents of URDF Files and Launch ROS Files

Never use any other character in these files instead of english language include.

## XML Files
## robot.xacro:
    <?xml version="1.0"?>
    <!-- #################################### -->
    <!-- DESCRIPTION OF THE TWO WHEELED ROBOT -->
    <!-- Made by Berat Tezer   -  February 24 -->
    <!-- #################################### -->

    <robot name="two_wheeled_drive_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
        <!-- Body Dimensions -->
        <xacro:property name="body_link_x_dim" value="0.144"/>
        <xacro:property name="body_link_y_dim" value="0.113"/>
        <xacro:property name="body_link_z_dim" value="0.030"/>

        <!-- Wheel Dimensions -->
        <xacro:property name="wheel_link_radius" value="0.045"/>
        <xacro:property name="wheel_link_length" value="0.1"/>
        <xacro:property name="wheel_link_z_location" value="0.0225"/>

        <!-- Upper Body Dimensions -->
        <xacro:property name="upper_body_link_x_dim" value="0.055"/>
        <xacro:property name="upper_body_link_y_dim" value="0.080"/>
        <xacro:property name="upper_body_link_z_dim" value="0.010"/>

        <!-- Upper Body Support Dimensions -->
        <xacro:property name="upper_body_support_link_x_dim" value="0.010"/>
        <xacro:property name="upper_body_support_link_y_dim" value="0.010"/>
        <xacro:property name="upper_body_support_link_z_dim" value="0.030"/>

        <!-- Support Wheel Body Dimension -->
        <xacro:property name="support_wheel_body_link_x_dim" value="0.020"/>
        <xacro:property name="support_wheel_body_link_y_dim" value="0.020"/>
        <xacro:property name="support_wheel_body_link_z_dim" value="0.010"/>

        <!-- Support Wheel Dimension -->
        <xacro:property name="support_wheel_link_radius" value="0.0050"/>
        <xacro:property name="support_wheel_link_length" value="0.1"/>
        <xacro:property name="support_wheel_link_z_location" value="0.010"/>

        <!-- Material Density -->
        <xacro:property name="body_density" value="2710.0"/>
        <xacro:property name="wheel_density" value="2710.0"/>

        <!-- Pi constant -->
        <xacro:property name="pi_const" value="3.14159265"/>
        
        <!-- Robot body and wheel mass -->
        <xacro:property name="body_mass" value="${body_density*body_link_x_dim*body_link_y_dim*body_link_z_dim}"/>
        <xacro:property name="wheel_mass" value="${wheel_density*pi_const*wheel_link_radius*wheel_link_radius*wheel_link_length}"/>
        <xacro:property name="upper_body_mass" value="${body_density*upper_body_link_x_dim*upper_body_link_y_dim*upper_body_link_z_dim}"/>
        <xacro:property name="upper_body_support_mass" value="${body_mass*upper_body_support_link_x_dim*upper_body_support_link_y_dim*upper_body_support_link_z_dim}"/>
        <xacro:property name="support_wheel_body_mass" value="${body_mass*support_wheel_body_link_x_dim*support_wheel_body_link_y_dim*support_wheel_body_link_z_dim}"/>
        <xacro:property name="support_wheel_mass" value="${wheel_density*pi_const*(support_wheel_link_radius/2.0)*(support_wheel_link_radius/2.0)*support_wheel_link_length}"/>
        
        <!-- Moments of inertia of the wheel -->
        <xacro:property name="Iz_wheel" value="${0.5*wheel_mass*wheel_link_radius*wheel_link_radius}"/>
        <xacro:property name="I_wheel" value="${(1.0/12.0)*wheel_mass*(3.0*wheel_link_radius/wheel_link_radius+wheel_link_length*wheel_link_length)}"/>

        <!-- This macro defines the complete inertial section of the wheel -->
        <!-- It is used later in cod -->
        <xacro:macro name="inertia_wheel">
            <inertial>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <mass value="${wheel_mass}"/>
                <inertia ixx="${I_wheel}" ixy="0.0" ixz="0.0" iyy="${I_wheel}" iyz="0.0" izz="${Iz_wheel}"/>
            </inertial>
        </xacro:macro>

        <xacro:macro name="inertia_upper_body_support">
            <inertial>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <mass value="${upper_body_support_mass}"/>
                <inertia ixx="${(1.0/12.0)*upper_body_support_mass*(upper_body_support_link_y_dim*upper_body_support_link_y_dim+upper_body_support_link_z_dim*upper_body_support_link_z_dim)}" ixy="0" ixz="0" iyy="${(1.0/12.0)*upper_body_support_mass*(upper_body_support_link_x_dim*upper_body_support_link_x_dim+upper_body_support_link_z_dim*upper_body_support_link_z_dim)}" iyz="0" izz="${(1.0/12.0)*upper_body_support_mass*(upper_body_support_link_x_dim*upper_body_support_link_x_dim+upper_body_support_link_y_dim*upper_body_support_link_y_dim)}"/>
            </inertial>
        </xacro:macro>

        <xacro:macro name="inertia_upper_body">
            <inertial>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <mass value="${upper_body_mass}"/>
                <inertia ixx="${(1.0/12.0)*upper_body_mass*(upper_body_link_y_dim*upper_body_link_y_dim+upper_body_link_z_dim*upper_body_link_z_dim)}" ixy="0" ixz="0" iyy="${(1.0/12.0)*upper_body_mass*(upper_body_link_x_dim*upper_body_link_x_dim+upper_body_link_z_dim*upper_body_link_z_dim)}" iyz="0" izz="${(1.0/12.0)*upper_body_mass*(upper_body_link_x_dim*upper_body_link_x_dim+upper_body_link_y_dim*upper_body_link_y_dim)}"/>
            </inertial>
        </xacro:macro>

        <!-- Over here we include the file that defines extra Gazebo options -->
        <!-- and motion control driver -->
        <xacro:include filename="$(find robot_model_pkg)/urdf/robot.gazebo"/>

        <!-- ############################## -->
        <!-- Now we define Links and Joints -->
        <!-- ############################## -->

        <!-- In ROS, created objects calls by links -->
        <!-- Joints are coming meaning that connections between two different shapes -->
        <!-- We won't call these connections as links -->

        <!-- We need to have this link, otherwise Gazebo will complain -->
        <link name="base"></link>
        <joint name="base_joint" type="fixed">
            <parent link="base"/>
            <child link="body_link"/>
        </joint>

        <!-- ################# -->
        <!-- Body of our robot -->
        <!-- ################# -->

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
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            </collision>

            <inertial>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <mass value="${body_mass}"/>
                <inertia ixx="${(1.0/12.0)*body_mass*(body_link_y_dim*body_link_y_dim+body_link_z_dim*body_link_z_dim)}" ixy="0" ixz="0" iyy="${(1.0/12.0)*body_mass*(body_link_x_dim*body_link_x_dim+body_link_z_dim*body_link_z_dim)}" iyz="0" izz="${(1.0/12.0)*body_mass*(body_link_y_dim*body_link_y_dim+body_link_x_dim*body_link_x_dim)}"/>
            </inertial>
        </link>

        <!-- ################### -->
        <!-- Wheels of our robot -->
        <!-- ################### -->

        <!-- Right Wheel -->
        <joint name="wheel_right_joint" type="continuous">
            <parent link="body_link"/>
            <child link="wheel_right_link"/>
            <origin xyz="${-body_link_x_dim/2+1.2*wheel_link_radius} ${-body_link_y_dim/2-wheel_link_length/2} ${wheel_link_z_location}" rpy="0.0 0.0 0.0"/>
            <axis xyz="0.0 1.0 0.0"/>
            <limit lower="0.0" upper="0.0" effort="1000.0" velocity="1000.0"/>
            <dynamics damping="1.0" friction="1.0"/>
        </joint>

        <link name="wheel_right_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
                <geometry>
                    <cylinder radius="${wheel_link_radius}" length="${wheel_link_length}"/>
                </geometry>
            </visual>

            <collision>
                <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
                <geometry>
                    <cylinder radius="${wheel_link_radius}" length="${wheel_link_length}"/>
                </geometry>
            </collision>

            <!-- Macro we defined before -->
            <xacro:inertia_wheel/>
        </link>

        <!-- Left Wheel -->
        <joint name="wheel_left_joint" type="continuous">
            <parent link="body_link"/>
            <child link="wheel_left_link"/>
            <origin xyz="${-body_link_x_dim/2+1.2*wheel_link_radius} ${body_link_y_dim/2+wheel_link_length/2} ${wheel_link_z_location}" rpy="0.0 0.0 0.0"/>
            <axis xyz="0.0 1.0 0.0"/>
            <limit lower="0.0" upper="0.0" effort="1000.0" velocity="1000.0"/>
            <dynamics damping="1.0" friction="1.0"/>
        </joint>

        <link name="wheel_left_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
                <geometry>
                    <cylinder radius="${wheel_link_radius}" length="${wheel_link_length}"/>
                </geometry>
            </visual>

            <collision>
                <origin xyz="0.0 0.0 0.0" rpy="1.570795 0.0 0.0"/>
                <geometry>
                    <cylinder radius="${wheel_link_radius}" length="${wheel_link_length}"/>
                </geometry>
            </collision>

            <!-- Macro we defined before -->
            <xacro:inertia_wheel/>
        </link>

        <!-- Support Wheel Body -->
        <joint name="support_wheel_body_joint" type="fixed">
            <parent link="body_link"/>
            <child link="support_wheel_body_link"/>
            <origin xyz="0.015 0.0 0.047" rpy="0.0 0.0 0.0"/>
            <axis xyz="0.0 0.0 0.0"/>
            <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
            <dynamics damping="0.0" friction="0.0"/>
        </joint>

        <link name="support_wheel_body_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.020 0.020 0.010"/>    
                </geometry>
            </visual>

            <collision>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.020 0.020 0.010"/>    
                </geometry>
            </collision>
        </link>

        <!-- Support Wheel -->
        <joint name="support_wheel_joint" type="continuous">
            <parent link="support_wheel_body_link"/>
            <child link="support_wheel_link"/>
            <origin xyz="-0.047 0.0 0.015" rpy="0.0 0.0 0.0"/>
            <axis xyz="0.0 0.0 0.0"/>
            <limit lower="0.0" upper="0.0" effort="1000.0" velocity="1000.0"/>
            <dynamics damping="1.0" friction="1.0"/>
        </joint>

        <link name="support_wheel_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="1.2345 0.56789 0.392699"/>
                <geometry>
                    <sphere radius="${support_wheel_link_radius}"/>
                </geometry>
            </visual>
                
            <collision>
                <origin xyz="0.0 0.0 0.0" rpy="1.2345 0.56789 0.392699"/>
                <geometry>
                    <sphere radius="${support_wheel_link_radius}"/>
                </geometry>
            </collision>
            
            <inertial>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <mass value="${body_mass}"/>
                <inertia ixx="${(2.0/5.0)*support_wheel_mass*support_wheel_link_radius*support_wheel_link_radius}" ixy="0" ixz="0" iyy="${(2.0/5.0)*support_wheel_mass*0.005*0.005}" iyz="0" izz="${(2.0/5.0)*support_wheel_mass*0.005*0.005}"/>
            </inertial>
        </link>

        <!-- ###################################### -->
        <!-- First four support sticks of our robot -->
        <!-- ###################################### -->

        <!-- Left front -->
        <joint name="upper_body_support_lf_joint" type="fixed">
            <parent link="body_link"/>
            <child link="upper_body_support_lf_link"/>
            <origin xyz="-0.0155 -0.033 0.04" rpy="0.0 0.0 0.0"/>
            <axis xyz="0.0 0.0 0.0"/>
            <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
        </joint>

        <link name="upper_body_support_lf_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.01 0.01 0.03"/>
                </geometry>
            </visual>

            <collision>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.01 0.01 0.03"/>
                </geometry>
            </collision>

            <!-- Macro we defined before -->
            <xacro:inertia_upper_body_support/>
        </link>

        <!-- Left back -->
        <joint name="upper_body_support_lb_joint" type="fixed">
            <parent link="body_link"/>
            <child link="upper_body_support_lb_link"/>
            <origin xyz="-0.0565 -0.033 0.04" rpy="0.0 0.0 0.0"/>
            <axis xyz="0.0 0.0 0.0"/>
            <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
        </joint>

        <link name="upper_body_support_lb_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.01 0.01 0.03"/>
                </geometry>
            </visual>

            <collision>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.01 0.01 0.03"/>
                </geometry>
            </collision>

            <!-- Macro we defined before -->
            <xacro:inertia_upper_body_support/>
        </link>

        <!-- Right front -->
        <joint name="upper_body_support_rf_joint" type="fixed">
            <parent link="body_link"/>
            <child link="upper_body_support_rf_link"/>
            <origin xyz="-0.0155 0.033 0.04" rpy="0.0 0.0 0.0"/>
            <axis xyz="0.0 0.0 0.0"/>
            <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
        </joint>

        <link name="upper_body_support_rf_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.01 0.01 0.03"/>
                </geometry>
            </visual>

            <collision>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.01 0.01 0.03"/>
                </geometry>
            </collision>

            <!-- Macro we defined before -->
            <xacro:inertia_upper_body_support/>
        </link>

        <!-- Right back -->
        <joint name="upper_body_support_rb_joint" type="fixed">
            <parent link="body_link"/>
            <child link="upper_body_support_rb_link"/>
            <origin xyz="-0.0565 0.033 0.04" rpy="0.0 0.0 0.0"/>
            <axis xyz="0.0 0.0 0.0"/>
            <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
        </joint>

        <link name="upper_body_support_rb_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.01 0.01 0.03"/>
                </geometry>
            </visual>

            <collision>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.01 0.01 0.03"/>
                </geometry>
            </collision>

            <!-- Macro we defined before -->
            <xacro:inertia_upper_body_support/>
        </link>

        <!-- ############################# -->
        <!-- First upper body of our robot -->
        <!-- ############################# -->

        <joint name="upper_body_1_joint" type="fixed">
            <parent link="upper_body_support_lf_link"/>
            <child link="upper_body_1_link"/>
            <origin xyz="-0.036 0.0 0.075" rpy="0.0 0.0 0.0"/>
            <axis xyz="0.0 0.0 0.0"/>
            <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
        </joint>

        <link name="upper_body_1_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.055 0.08 0.01"/>
                </geometry>
            </visual>

            <collision>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.055 0.08 0.01"/>
                </geometry>
            </collision>
            
            <!-- Macro we defined before -->
            <xacro:inertia_upper_body/>
        </link>

        <!-- ####################################### -->
        <!-- Second four support sticks of our robot -->
        <!-- ####################################### -->

        <!-- Left front -->
        <joint name="upper_body_support_lf_2_joint" type="fixed">
            <parent link="upper_body_1_link"/>
            <child link="upper_body_support_lf_2_link"/>
            <origin xyz="-0.0155 -0.033 0.08" rpy="0.0 0.0 0.0"/>
            <axis xyz="0.0 0.0 0.0"/>
            <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
        </joint>

        <link name="upper_body_support_lf_2_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.01 0.01 0.03"/>
                </geometry>
            </visual>

            <collision>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.01 0.01 0.03"/>
                </geometry>
            </collision>

            <!-- Macro we defined before -->
            <xacro:inertia_upper_body_support/>
        </link>

        <!-- Left back -->
        <joint name="upper_body_support_lb_2_joint" type="fixed">
            <parent link="upper_body_1_link"/>
            <child link="upper_body_support_lb_2_link"/>
            <origin xyz="-0.0565 -0.033 0.08" rpy="0.0 0.0 0.0"/>
            <axis xyz="0.0 0.0 0.0"/>
            <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
        </joint>

        <link name="upper_body_support_lb_2_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.01 0.01 0.03"/>
                </geometry>
            </visual>

            <collision>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.01 0.01 0.03"/>
                </geometry>
            </collision>

            <!-- Macro we defined before -->
            <xacro:inertia_upper_body_support/>
        </link>

        <!-- Right front -->
        <joint name="upper_body_support_rf_2_joint" type="fixed">
            <parent link="upper_body_1_link"/>
            <child link="upper_body_support_rf_2_link"/>
            <origin xyz="-0.0155 0.033 0.08" rpy="0.0 0.0 0.0"/>
            <axis xyz="0.0 0.0 0.0"/>
            <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
        </joint>

        <link name="upper_body_support_rf_2_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.01 0.01 0.03"/>
                </geometry>
            </visual>

            <collision>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.01 0.01 0.03"/>
                </geometry>
            </collision>

            <!-- Macro we defined before -->
            <xacro:inertia_upper_body_support/>
        </link>

        <!-- Right back -->
        <joint name="upper_body_support_rb_2_joint" type="fixed">
            <parent link="upper_body_1_link"/>
            <child link="upper_body_support_rb_2_link"/>
            <origin xyz="-0.0565 0.033 0.08" rpy="0.0 0.0 0.0"/>
            <axis xyz="0.0 0.0 0.0"/>
            <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
        </joint>

        <link name="upper_body_support_rb_2_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.01 0.01 0.03"/>
                </geometry>
            </visual>

            <collision>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.01 0.01 0.03"/>
                </geometry>
            </collision>

            <!-- Macro we defined before -->
            <xacro:inertia_upper_body_support/>
        </link>

        <!-- ############################## -->
        <!-- Second upper body of our robot -->
        <!-- ############################## -->

        <joint name="upper_body_2_joint" type="fixed">
            <parent link="upper_body_support_lf_2_link"/>
            <child link="upper_body_2_link"/>
            <origin xyz="-0.036 0.0 0.15" rpy="0.0 0.0 0.0"/>
            <axis xyz="0.0 0.0 0.0"/>
            <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
        </joint>

        <link name="upper_body_2_link">
            <visual>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.055 0.08 0.01"/>
                </geometry>
            </visual>

            <collision>
                <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                <geometry>
                    <box size="0.055 0.08 0.01"/>
                </geometry>
            </collision>
            
            <!-- Macro we defined before -->
            <xacro:inertia_upper_body/>
        </link>
    </robot>

## robot.gazebo:
    <?xml version="1.0"?>
    <!-- ###################################################### -->
    <!-- GAZEBO ADDITIONAL DESCRIPTION OF THE TWO WHEELED ROBOT -->
    <!-- Made by Berat Tezer           -            February 24 -->
    <!-- ###################################################### -->

    <robot>
        <!-- http://classic.gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros -->
        
        <!-- mu1 and mu2 are friction coefficients -->
        <gazebo reference="body_link">
            <mu1>0.2</mu1>
            <mu2>0.2</mu2>
            <material>Gazebo/Gray</material>
        </gazebo>

        <gazebo reference="wheel_right_link">
            <mu1>0.2</mu1>
            <mu2>0.2</mu2>
            <material>Gazebo/Yellow</material>
        </gazebo>

        <gazebo reference="wheel_left_link">
            <mu1>0.2</mu1>
            <mu2>0.2</mu2>
            <material>Gazebo/Yellow</material>
        </gazebo>

        <gazebo reference="support_wheel_body_link">
            <mu1>0.2</mu1>
            <mu2>0.2</mu2>
            <material>Gazebo/Gray</material>
        </gazebo>

        <gazebo reference="support_wheel_link">
            <mu1>0.2</mu1>
            <mu2>0.2</mu2>
            <material>Gazebo/Yellow</material>
        </gazebo>

        <gazebo reference="upper_body_support_lf_link">
            <mu1>0.2</mu1>
            <mu2>0.2</mu2>
            <material>Gazebo/Green</material>
        </gazebo>

        <gazebo reference="upper_body_support_lb_link">
            <mu1>0.2</mu1>
            <mu2>0.2</mu2>
            <material>Gazebo/Green</material>
        </gazebo>

        <gazebo reference="upper_body_support_rf_link">
            <mu1>0.2</mu1>
            <mu2>0.2</mu2>
            <material>Gazebo/Green</material>
        </gazebo>

        <gazebo reference="upper_body_support_rb_link">
            <mu1>0.2</mu1>
            <mu2>0.2</mu2>
            <material>Gazebo/Green</material>
        </gazebo>

        <gazebo reference="upper_body_1_link">
            <mu1>0.2</mu1>
            <mu2>0.2</mu2>
            <material>Gazebo/Gray</material>
        </gazebo>

        <gazebo reference="upper_body_support_lf_2_link">
            <mu1>0.2</mu1>
            <mu2>0.2</mu2>
            <material>Gazebo/Green</material>
        </gazebo>

        <gazebo reference="upper_body_support_lb_2_link">
            <mu1>0.2</mu1>
            <mu2>0.2</mu2>
            <material>Gazebo/Green</material>
        </gazebo>

        <gazebo reference="upper_body_support_rf_2_link">
            <mu1>0.2</mu1>
            <mu2>0.2</mu2>
            <material>Gazebo/Green</material>
        </gazebo>

        <gazebo reference="upper_body_support_rb_2_link">
            <mu1>0.2</mu1>
            <mu2>0.2</mu2>
            <material>Gazebo/Green</material>
        </gazebo>

        <gazebo reference="upper_body_support_rb_2_link">
            <mu1>0.2</mu1>
            <mu2>0.2</mu2>
            <material>Gazebo/Gray</material>
        </gazebo>
        
        <!-- Controller for the four wheeled robot -->
        <gazebo>
            <plugin name="skid_steer_drive_controller" filename="libgazebo_ros_skid_steer_drive.so">

                <!-- Control update raze in Hz -->
                <updateRate>100.0</updateRate>        
                
                <!-- Leave this empty otherwise there will be problems with sending control commands -->
                <robotNamespace></robotNamespace>
                
                <!-- Robot Kinematics -->
                <leftBackJoint>wheel_left_joint</leftBackJoint>
                <rightBackJoint>wheel_right_joint</rightBackJoint>
                <supportWheelBodyJoint>support_wheel_body_joint</supportWheelBodyJoint>
                <supportWheelJoint>support_wheel_joint</supportWheelJoint>
                <upperBodySupportLf>upper_body_support_lf_joint</upperBodySupportLf>
                <upperBodySupportLb>upper_body_support_lb_joint</upperBodySupportLb>
                <upperBodySupportRf>upper_body_support_rf_joint</upperBodySupportRf>
                <upperBodySupportRb>upper_body_support_rb_joint</upperBodySupportRb>
                <upperBody1>upper_body_1_joint</upperBody1>
                <upperBodySupportLf2>upper_body_support_lf_2_joint</upperBodySupportLf2>
                <upperBodySupportLb2>upper_body_support_lb_2_joint</upperBodySupportLb2>
                <upperBodySupportRf2>upper_body_support_rf_2_joint</upperBodySupportRf2>
                <upperBodySupportRb2>upper_body_support_rb_2_joint</upperBodySupportRb2>
                <upperBody2>upper_body_2_joint</upperBody2>
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

## robot_xacro.launch:
    <?xml version="1.0"?>
    <!-- ############################################################## -->
    <!-- LAUNCH FILE FOR THE GAZEBO SIMULATION OF THE TWO WHEELED ROBOT -->
    <!-- Made by Berat Tezer                -               February 24 -->
    <!-- ############################################################## -->

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
