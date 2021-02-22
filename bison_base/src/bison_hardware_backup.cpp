#include "bison_base/bison_hardware.h"
#include "controller_manager/controller_manager.h"
#include <ros/console.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bison");
    ros::NodeHandle nh;
    Bison robot;
    controller_manager::ControllerManager cm(&robot, nh);

    ROS_INFO_STREAM("1111111");
    while (true)
    {
        robot.read();
        cm.update(robot.getTime(), robot.getPeriod());
        robot.write();
        ros::Duration(1.0).sleep();
    }
}