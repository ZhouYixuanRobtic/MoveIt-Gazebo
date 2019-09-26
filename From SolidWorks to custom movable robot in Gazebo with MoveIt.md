## From SolidWorks to custom movable robot in Gazebo with MoveIt

​	This tutorial aims at providing a propagable path from SolidWorks to  a movable robot in Gazebo with MoveIt. A complete example could be found  [here]()

​	First We wanna list most important components versions out, they are SolidWorks 2016 + Ubuntu 16.04 + ROS kinetic + Gazebo 9.0 + MoveIt. Some other components like `ros_control` and `sw_urdf_exporter` will be mentioned when needed. Before we start, you should already have a proper SolidWorks model.

[From SolidWorks to custom movable robot in Gazebo with MoveIt](##From From SolidWorks to custom movable robot in Gazebo with MoveIt )

​	[1. Convert SolidWorks model to URDF](#1-convert-solidworks-model-to-urdf)

​	[2. Add controllers and everything needed in URDF](#2-add-controllers-and-everything-needed-in-urdf)

​	[3. Gazebo Control](#3-gazebo-control)

​	[4. MoveIt configuration](#4-moveit-configuration)

​	[5. Check your MoveIt setup under Rviz](#5-check-your-moveit-setup-under-rviz)

​	[6. Use Gazebo + Rviz to simulate](#6-use-gazebo--rviz-to-simulate)

​	[7. Issues and Solutions](#7-issues-and-solutions)


### 1 .Convert SolidWorks model to URDF

​	[URDF](<http://wiki.ros.org/urdf>) is an XML format for representing a robot model. We use *URDF* to spawn robot *RVIZ* and *GAZEBO*.

- **Export**

  `sw_urdf_exporter` is a *SolidWorks* add-in that allows for the convenient export of *SW* Parts and Assemblies into a *URDF* file and creates a *ROS* package. Instructions on how to utilize this add-in can be found on its [wiki](http://wiki.ros.org/sw_urdf_exporter) page. Tips below may help based on our practice.

  - **Model Simplification**

    `sw_urdf_exporter` converts a *SolidWorks* model into *STL* files which contain geometry information of each link and *URDF* file which records  the relative relationship between links. Of course, a complicated model generates heavy *STL* files and hence causes high computation burdens. To alleviate this issue, we should have our model simplified first. Without a doubt, if you have an adequately simple model, just let it be.

    - Divide your assembly into parts refer to link partition you needed. Record mass, inertia matrix, the center of gravity and other physic properties,  to have  a higher simulation performance by modifying *URDF* generated. When you recording your inertia matrix, noticing that if the direction of  the reference axis matches your imagination. If not, new a different reference coordinates system.
    - Every part of Assembly should be saved as a Part. Make the mass as same as the original Assembly part is by setting the material.
    - Re-assemble your Parts. Remember you don't need interior structure, so be bold to delete things like a screw, just keep it's exterior looking the same.

  - **Initial Pose**

    In the process of exporting, the initial pose of the model will be considered as the reference of *URDF*. So try to put your model at a stable position even with gravity. 
    
  - **Name it regularly**

    Please do not name your joints as `joint1` or something, try to name them regularly for the sake of indeed readability.  Names as`shoulder_joint` and so on are recommended.

  - **Issues may exist and their solutions**

    - Rviz takes too long to import the model, and even gives a timeout error.

      Generally, the model's large size should take responsibility. So simplify your model.

    - Exported model performs malposition and changed geometry relationships such as  declination of two coaxial solid of revolution.

      May the exporter gets wrong on account of the model's some complicated characteristics. Again, simplify your model.

    - Model fracture or wrong assemble relationship

      Something wrong with the set of Frame and Axis. We do recommend that axes of  child links should be set under parent links, and frames should be set under child links themselves. Besides, the axis should go through the frame's origin and be parallel or coincident to one of three basic axes (XYZ).

-  **Dummy Link**

  *MoveIt* does not support a root link with inertia, so an extra dummy link for your *URDF* is necessary. Open your *URDF* file, and add content below after `/robot` tag.

  ```xml
  <link name="world"/>
  <joint
      name="base_connect2_world"
      type="fixed">
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <parent
        link="world" />
      <child
        link="base_link" />
  </joint>
  ```

- **Check your URDF in *RVIZ* and *GAZEBO***

​	Set up a workspace named `catkin_ws`or  name it whatever you like, and use catkin tools to initialize it. Put your generated package in your workspace and compile it. Run `roslaunch RobotName display.launch ` to open *RVIZ* and `roslaunch RobotName gazebo.launch` to open *GAZEBO*. The folder `urdf` should be renamed as `robots` and a legal email address under the `package.xml` file is required too.

​	 Check your *URDF* file, focus on joint revolute direction, limit and the initial position of links. If you find anything weird, try to fix it under the instructions of chapter 7.  **Warning: A proper URDF is the foundation of this tutorial so must check your generated URDF carefully **

​	**Notice: From now on,  the following uses of `catkin_ws` refer to your own workspace. Similarly, `RobotName` is used to replace your own robot name **

### 2. Add controllers and everything needed in URDF

​	 The URDF generated by aforementioned `sw_urdf_exporter` is not appropriate for  a Gazebo simulation. You are required  to add `transmission` and `plugin`. First of all, the `plugin` must be appended after your model description, and modify the `RobotName` into your own robot name.
```XML
<gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/RobotName</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    <legacyModeNS>true</legacyModeNS>
    </plugin>
 </gazebo>
```

 	The transmissions are inevitable, all joints must have a correspondent transmission label like 

```xml
<transmission name="Joint1_trans">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="Joint1">        					          		  <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      </joint>
      <actuator name="Joint1_motor">
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
</transmission>
```

Again does not forget to modify your own joint name. Here the `hardwareInterface` could be set as `Position`, `Velocity` and `Effort`. Generally, `Position ` is a good choice, while please feel free to modify it according to your requirements.

### 3. Gazebo Control

​	The control package is used to spawn all controllers and publish some topics needed and depend on the `ros_contorl` and `gazebo_ros_control`. So before we begin this step, make sure that you've installed these packages. Use the second line under Gazebo9

​	Try the command below to get `gazebo_ros_control`

```c
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-gazebo9-ros-pkgs ros-kinetic-gazebo9-ros-control
```

​	Try the command below to get `ros_control`

```
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
```

​	Since the two packages were installed properly, we can go on with our Gazebo Control now. First we should create a configuration file named `RobotName_gazebo_control.yaml`.

```
cd ~/catkin_ws/src/RobotName/config
gedit RobotName_gazebo_control.yaml
```

 and copy the content below into it.  Don't forget to replace `Joint1` and `RobotName` with your own correct names.

```yaml
RobotName:
    #publish all joint states
    joint_state_controller:
        type: joint_state_controller/JointStateController
        publish_rate: 50
    RobotName_arm_controller:
        type: velocity_controllers/JointTrajectoryController
        joints:
            - Joint1
            - Joint2
            - Joint3
            - Joint4
            - Joint5
            - Joint6
        constraints:
            goal_time: &goal_time_constraint 4.0
            stopped_velocity_tolerance: 0.05
            Joint1:
                goal: &goal_pos_constraint 0.1
                trajectory: &trajectory_pos_constraint 0.1
            Joint2:
                goal: *goal_pos_constraint
                trajectory: *trajectory_pos_constraint
            Joint3:
                goal: *goal_pos_constraint
                trajectory: *trajectory_pos_constraint
            Joint4:
                goal: *goal_pos_constraint
                trajectory: *trajectory_pos_constraint
            Joint5:
                goal: *goal_pos_constraint
                trajectory: *trajectory_pos_constraint            
            Joint6:
                goal: *goal_pos_constraint
                trajectory: *trajectory_pos_constraint   
        stop_trajectory_duration: 1.0
        action_monitor_rate: 10
```

​	Then we should create our own launch file named `RobotName_simulation.launch`

```
cd ~/catkin_ws/src/RobotName/launch
gedit RobotName_simulation.launch
```

copy the content below. Please modify `****` and `$(arg RobotName)`.

```xml
<launch>
  <arg name="RobotName" default="****"/>
  <param name="robot_description" textfile="$(find RobotName)/robots/$(arg RobotName).urdf"/>
  <!-- 1.startup Gazebo -->
  <!-- these are the arguments you can pass this launch file, for example paused:=true -->
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

  <!-- We resume the logic in empty_world.launch -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="debug" value="$(arg debug)" />
    <arg name="gui" value="$(arg gui)" />
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="headless" value="$(arg headless)"/>
  </include>


  <!-- 2.Run a python script to the send a service call to gazebo_ros to spawn a URDF robot -->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen" args="-urdf -model $(arg RobotName) -param robot_description"/> 		

	<!-- 将Gazebo关节控制器的配置参数加载到参数服务器中 -->
  <rosparam file="$(find RobotName)/config/$(arg RobotName)_gazebo_control.yaml" command="load"/>
							     

  <!-- 加载控制器 -->
	<node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
                                        output="screen" ns="/$(arg RobotName)" args="joint_state_controller $(arg RobotName)_arm_controller"/>
 
</launch>
```

Save it and Gazebo Control is setup properly.  Run `roslaunch RobotName RobotName_simulation.launch`

### 4. MoveIt configuration

 	*MoveIt* is a motion planning framework which helps us to make a useful plan through a certain target. The official *MoveIt* tutorial is beginner-friendly, so we do recommend to read the official *MoveIt* [tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html). Under the purpose that we want to use *MoveIt* to accomplish the simulation with Gazebo, we are not only required to provide a complete  *MoveIt* configuration package but also expected to give some new configuration files. We'll begin with the official *MoveIt* configuration. Till now, the best kinematic solver supported by MoveIt is `trac_ik`, use `sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin` to install it.

```
roslaunch moveit_setup_assistant setup_assistant.launch
```

- **start**

Click `Create_New MoveIt Configuration Package` then`Browse`，choose your own *URDF* file（located at `~/catkin_ws/src/RobotName/RobotName/robots/RobotName.urdf`) and load it.

- **self-collisions**

The Default Self-Collision Matrix Generator searches for pairs of links on the robot that can safely be disabled from collision checking, decreasing motion planning processing time. These pairs of links are disabled when they are always in collision, never in collision, in collision in the robot's default position or when the Links are adjacent to each other on the kinematic chain. The sampling density specifies how many random robot positions to check for self collision. Higher densities require more computation time while lower densities have a higher possibility of disabling pairs that should not be disabled. The default value is 10,000 collision checks. Collision checking is done in parallel to decrease processing time.

- **virtual joints**

In most situations, we need a virtual joint to describe the robot's position in `/world` coordinate system.

1. click `add virtual joints`
2. name `virtual joint name` as virtual_joint，set `child link`as `world`,`parent frame name` as`world`,`joint type` as`fixed`。**Note: the parent frame is the dummy link we gave, and just in coincidence its name is world, you could name it whatever you like in the first chapter**
3. `save`

- **planning groups** 

Planning groups are used for semantically describing different parts of your robot, such as defining what an arm is, or an end effector. It integrate multiple links and joints into one groups. 

We recommend to add joints in a `kin.chain` (kinematic chain) way for tandem part (such as a arm(6 DOF) or a finger(3 DOF)) and other ways for non-tandem part. Besides, the best kinematic solver is `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` and the best planner is `RRTConnect`.

Built on our practice, set up a end-effector group separately can be helpful. Modify your `URDF` like,

```xml
  <link name="ee_link"/>
  <joint
    name="virtual_ee_mount_joint"
    type="fixed">
    <origin
      xyz="0.00 -0.097 0.0"
      rpy="0 0 0" />
    <parent
      link="palm_Link" />
    <child
      link="ee_link" />
  </joint>
```

**Noted: the ee_link here is a virtual and no-volume link,  set the joint value according to actual demands, for example the tip of a finger**

- robot poses

  set any robot poses you like.

- end effectors

  add your end effector like 

- Passive Joints and others

  add them when need.

 - configuration files

   Export your package named as `RobotName_moveit_config` after add your *Author Information*.

​	Run `demo.launch` to visualize it.

~~~c
roslaunch robotname_moveit demo.launch
~~~

### 5. Check your MoveIt setup under Rviz

- Read the `MoveIt!Quickstart in Rviz` part of the [official tutorials](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html) .

  1. In the navigation bar , choose `panels`click`add new panel`, add`RvizVisualToolsGui`.
  2. In the navigation bar , choose `panels`click`add new panel`, add `MotionPlanning-Trajectory Slider`.
  3. `Display`>`MotionPlanning`>`Planning Request`>`Planning Group`choose planning groups。check `Query Start State`,`Query Goal State`.
  4. `Display`>`MotionPlanning`>`Scene Robot`>`Show Robot Visual`,cancel it.
  5. `Display`>`MotionPlanning`>`Planned Path`>`Show Robot Visual`, enable it.

  In the `MotionPlanning`part，click`Planning`, use `update` to get random valid target，click`plan` to plan，`execute` see if the whole thing right.


### 6. Use Gazebo + Rviz to simulate

- add a `controllers.yaml` file under `Robot_Name_moveit_config/config`.

  ```
  cd ~/catkin_ws/src/RobotName_moveit_config/config 
  gedit controllers.yaml
  ```

  give code below, check right names.

  ```yaml
  controller_list:
    - name: RobotName/RobotName_arm_controller
      action_ns: follow_joint_trajectory
      type: FollowJointTrajectory
      default: true
      joints:
        - Joint1
        - Joint2
        - Joint3
        - Joint4
        - Joint5
        - Joint6
  ```

- add a `moveit_planning_execution.launch` file under `RobotName_moveit_config/launch`.

  ```
  cd ~/catkin_ws/src/RobotName_moveit_config/launch
  gedit moveit_planning_execution.launch
  ```

  ```xml
  <?xml version="1.0"?>
  <launch>
    <!-- The planning and execution components of MoveIt! configured to run -->
    <!-- using the ROS-Industrial interface. -->
    <arg name="RobotName" default="***">
    <arg name="load_robot_description" default="false"/>
  
    <!-- the "sim" argument controls whether we connect to a Simulated or Real robot -->
    <!--  - if sim=false, a robot_ip argument is required -->
    <arg name="sim" default="true" />
    <arg name="robot_ip"/>
   
    <!-- load the robot_description parameter before launching ROS-I nodes -->
    <include file="$(find RobotName_moveit_config)/launch/planning_context.launch" >
      <arg name="load_robot_description" value="$(arg load_robot_description)" />
    </include>
  
   <!-- industrial_robot_simulator: accepts robot commands and reports status -->
    <node name="joint_state_publisher" pkg="joint_state_publisher"
          type="joint_state_publisher">
        <rosparam param="/source_list">[/$(arg RobotName)/joint_states]</rosparam>
    </node>
  
    <!-- publish the robot state (tf transforms) -->
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
  
    <include file="$(find RobotName_moveit_config)/launch/move_group.launch">
      <arg name="publish_monitored_planning_scene" value="true" />
    </include>
  
    <include file="$(find RobotName_moveit_config)/launch/moveit_rviz.launch">
      <arg name="config" value="true"/>
    </include>
  </launch>
  ```

- Modify `RobotName_moveit_controller_manager.launch.xml` under `RobotName_moveit_config/launch`

  ```xml
  <launch>
  <!-- Set the param that trajectory_execution_manager needs to find the controller plugin -->
    <arg name="moveit_controller_manager" default="moveit_simple_controller_manager/MoveItSimpleControllerManager" />
    <param name="moveit_controller_manager" value="$(arg moveit_controller_manager)"/>
  
    <!-- load controller_list -->
    <rosparam file="$(find RobotName_moveit)/config/controllers.yaml"/>
  
  </launch>
  ```

- Save and compile 

  Run 

  ```
  roslaunch RobotName RobotName_simulation.launch 
  ```

  ```
  roslaunch RobotName_moveit_config moveit_planning_execution.launch
  ```

  Use Rviz like chapter 5, then see if the robot in Gazebo moves as same as it is performed in Rviz.

### 7. Issues and Solutions

- **Error**

  If there's errors who are similar to  `[ERROR] [1569296403.251170827, 0.170000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/palm_joint`, don't be panic. 

  Actually, Gazebo will calculate PID gains if no pointed PID gains. If manual pointed PID gains are required, then open `RobotName_gazebo_control.yaml` to add some first level content:

  ```
  gazebo_ros_control/pid_gains:
    shoulder_joint: {p: 1.0, i: 0.0, d: 1.0}
    # More joints...
  ```

  **Warning:  the PID gains here should be well tunned which is such a complex progress, so just leave the error be **

- **Model Fracture**

  If the model fracture appears when you open Rviz or Gazebo, then turn to chapter 1 and set your reference coordinates system and reference axes carefully till a complete model shows up.
