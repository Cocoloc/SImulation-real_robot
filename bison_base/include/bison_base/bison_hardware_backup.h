#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/console.h>

class Bison : public hardware_interface::RobotHW
{
public:
  Bison()
  {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_right("right_wheel_joint", &pos[0], &vel[0], &eff[0]);
    joint_state_interface_.registerHandle(state_handle_right);

    hardware_interface::JointStateHandle state_handle_left("left_wheel_joint", &pos[1], &vel[1], &eff[1]);
    joint_state_interface_.registerHandle(state_handle_left);

    registerInterface(&joint_state_interface_);

    // connect and register the joint position interface
    hardware_interface::JointHandle velocity_handle_right(joint_state_interface_.getHandle("right_wheel_joint"), &cmd[0]);
    velocity_joint_interface_.registerHandle(velocity_handle_right);

    hardware_interface::JointHandle velocity_handle_left(joint_state_interface_.getHandle("left_wheel_joint"), &cmd[1]);
    velocity_joint_interface_.registerHandle(velocity_handle_left);

    registerInterface(&velocity_joint_interface_);
  }

  void read()
  {
    ROS_INFO_STREAM("reeeee");
  }
  void write()
  {
    ROS_INFO_STREAM("heeloo");
  }

  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.02); }

private:
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};