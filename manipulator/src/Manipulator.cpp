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
bool Manipulator::goRandomValid() const
{
    while(ros::ok())
    {
        move_group->setRandomTarget();
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if(move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            return move_group->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
        usleep(500000);
    }
}
Eigen::Affine3d Manipulator::getEndMotion(MultiplyType multiply_type, const Eigen::Vector3d & Translation, const Eigen::Vector3d & RPY)
{
    Eigen::Affine3d DesiredMotion;
    DesiredMotion.translation()=Translation;
    DesiredMotion.linear()=Eigen::Matrix3d{Eigen::AngleAxisd(RPY[2], Eigen::Vector3d::UnitZ())*
                                          Eigen::AngleAxisd(RPY[1], Eigen::Vector3d::UnitY())*
                                          Eigen::AngleAxisd(RPY[0], Eigen::Vector3d::UnitX())};
    Eigen::fromMsg(move_group->getCurrentPose().pose,Trans_B2E_);
    //left multiply or right multiply
    switch (multiply_type)
    {
        case RIGHT:
            return Trans_B2E_*DesiredMotion;
        case LEFT:
            return DesiredMotion*Trans_B2E_;
        default:
            return Trans_B2E_*DesiredMotion;
    }
}
bool Manipulator::linearMoveTo(const Eigen::Vector3d &destination_translation, double velocity_scale)
{
    static double linear_step = 0.01;
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    Eigen::Affine3d linear_destination=getEndMotion(RIGHT,destination_translation);
    moveit_msgs::OrientationConstraint pcm;
    pcm.link_name = EE_NAME;
    pcm.header.frame_id = move_group->getPlanningFrame();
    pcm.header.stamp = ros::Time::now();
    pcm.absolute_x_axis_tolerance = 0.001;
    pcm.absolute_y_axis_tolerance = 0.001;
    pcm.absolute_z_axis_tolerance = 0.001;
    pcm.orientation = move_group->getCurrentPose().pose.orientation;
    pcm.weight=1.0;
    moveit_msgs::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(pcm);
    move_group->setPathConstraints(path_constraints);
    move_group->setPoseTarget(linear_destination);
    move_group->setPlanningTime(10.0);
    bool success = move_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    move_group->setPlanningTime(5.0);
    move_group->clearPathConstraints();
    return success;
}
double Manipulator::allClose(const std::vector<double> & goal) const
{
    std::vector<double> current_joints{move_group->getCurrentJointValues()};
    std::vector<double>	auxiliary;

    std::transform (current_joints.begin(), current_joints.end(), goal.begin(), std::back_inserter(auxiliary),\
                    [](double element1, double element2) {return pow((element1-element2),2);});
    auxiliary.shrink_to_fit();
    return sqrt(std::accumulate(auxiliary.begin(), auxiliary.end(), 0.0));
}
void Manipulator::scale_trajectory_speed(moveit::planning_interface::MoveGroupInterface::Plan &plan,double scale_factor) const
{
    scale_factor = 20*scale_factor;
    for(auto & point : plan.trajectory_.joint_trajectory.points)
    {
        point.time_from_start *=1/scale_factor;
        for(int i=0; i<plan.trajectory_.joint_trajectory.joint_names.size();++i)
        {
            point.velocities[i] *= scale_factor;
            point.accelerations[i] *= scale_factor*scale_factor;
        }
    }
}