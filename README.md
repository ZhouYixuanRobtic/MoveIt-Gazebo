# MoveIt-Gazebo
​	From SolidWorks to custom movable robot in Gazebo with MoveIt

## Install

```
chmod +x environment.sh
./environmen.sh
```

## Tutorial
   ​See the [Tutorial](https://github.com/ZhouYixuanRobtic/MoveIt-Gazebo/blob/master/From%20SolidWorks%20to%20custom%20movable%20robot%20in%20Gazebo%20with%20MoveIt.md)
## Demo
   The folder named `manipulator` contains a simple demo makes the robot go random valid targets,
   but you need to specify the planning group name when you run this demo.
## Usage
```
  roslaunch aid aid_simulation.launch
  roslaunch aid_moveit_config moveit_planning_execution.launch
  rosrun moveit_manipulation moveit_manipulation aid_arm
```

OR

```
  roslaunch aidplus aidplus_simulation.launch
  roslaunch aidplus_moveit_config moveit_planning_execution.launch
  rosrun moveit_manipulation moveit_manipulation aidplus_arm
```

