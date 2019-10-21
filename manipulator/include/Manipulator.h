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
    std::string EE_NAME;
    moveit::planning_interface::MoveGroupInterface *move_group;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group;

    enum MultiplyType{
        RIGHT,
        LEFT,
    };
    Eigen::Affine3d Trans_B2E_;
    Eigen::Affine3d getEndMotion(MultiplyType multiply_type,
                                 const Eigen::Vector3d & Translation,
                                 const Eigen::Vector3d & RPY = Eigen::Vector3d(0.0, 0.0, 0.0));
    bool linearMoveTo(const Eigen::Vector3d & destination, double velocity_scale);
    double allClose(const std::vector<double> & goal) const;
    void scale_trajectory_speed(moveit::planning_interface::MoveGroupInterface::Plan & plan, double scale_factor) const;
public:
    Manipulator(const std::string & planning_group);
    ~Manipulator();
    void addStaticPlanningConstraint() const;
    bool goRandomValid() const;
};


#endif //MOVEIT_GAZEBO_MANIPULATOR_H
