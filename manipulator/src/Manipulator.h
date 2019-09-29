#ifndef MOVEIT_GAZEBO_MANIPULATOR_H
#define MOVEIT_GAZEBO_MANIPULATOR_H

#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include "ros/callback_queue.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

class Manipulator {
private:
    ros::NodeHandle n_;
    std::string _PLANNING_GROUP_;
    std::string EE_NAME{};
    moveit::planning_interface::MoveGroupInterface *move_group;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group;
public:
    Manipulator(const std::string & planning_group);
    ~Manipulator();
    void addStaticPlanningConstraint() const;
    void goRandomValid() const;
};


#endif //MOVEIT_GAZEBO_MANIPULATOR_H
