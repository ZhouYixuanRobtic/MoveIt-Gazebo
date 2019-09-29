#include "Manipulator.h"
Manipulator::Manipulator(const std::string &planning_group)
{
    this->_PLANNING_GROUP_=planning_group;
    move_group = new moveit::planning_interface::MoveGroupInterface(_PLANNING_GROUP_);
    planning_scene_interface = new  moveit::planning_interface::PlanningSceneInterface;
    joint_model_group = move_group->getCurrentState()->getJointModelGroup(_PLANNING_GROUP_);
    ROS_INFO_NAMED("Visual Servo", "Reference frame: %s", move_group->getPlanningFrame().c_str());
    ROS_INFO_NAMED("Visual Servo", "End effector link: %s", move_group->getEndEffectorLink().c_str());
    EE_NAME=move_group->getEndEffectorLink().c_str();
}
Manipulator::~Manipulator()
{
    delete move_group;
    delete planning_scene_interface;
    delete joint_model_group;
}
void Manipulator::addStaticPlanningConstraint() const
{
    std::vector<moveit_msgs::CollisionObject> objects;
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group->getPlanningFrame();
    collision_object.id = "ground";
    shape_msgs::Plane plane;
    plane.coef={0,0,1,0};
    geometry_msgs::Pose object_pose;
    object_pose.position.x=0;
    object_pose.position.y=0;
    object_pose.position.z=0.0;
    object_pose.orientation.w=1.0;
    collision_object.planes.push_back(plane);
    collision_object.plane_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;
    objects.push_back(collision_object);
    planning_scene_interface->addCollisionObjects(objects);
}
void Manipulator::goRandomValid() const
{
    while(ros::ok())
    {
        move_group->setRandomTarget();
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if(move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            move_group->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
            break;
        }
        usleep(500000);
    }
}