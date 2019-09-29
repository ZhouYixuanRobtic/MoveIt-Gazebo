#include "Manipulator.h"
int main(int argc, char* argv[])
{
    ros::init(argc,argv,"moveit_manipulation");
    if(argc<2)
    {
        ROS_INFO("No Planning Group input\r\n");
        return -1;
    }
    else
    {
        ros::AsyncSpinner spinner(1);
        spinner.start();
        Manipulator robot_manipulator(argv[1]);
        ROS_INFO("Manipulator initialized with %s\r\n",argv[1]);
        ros::Rate loop_rate(30);
        robot_manipulator.addStaticPlanningConstraint();
        while(ros::ok())
        {
            robot_manipulator.goRandomValid();
            loop_rate.sleep();
        }
        ros::waitForShutdown();
    }
    return 0;
}