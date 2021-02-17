#include "real_robot_base/my_robot_hardware.h"
main()
{
  MyRobot robot;
  controller_manager::ControllerManager cm(&robot);

  while (true)
  {
     robot.read();
     cm.update(robot.get_time(), robot.get_period());
     robot.write();
     sleep();
  }
}