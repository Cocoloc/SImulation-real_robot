#pragma once

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <bison_base/drive.h>
// ostringstream
#include <sstream>
#define M_PI 3.14159265358979323846 /* pi */

const unsigned int NUM_JOINTS = 2;

/// \brief Hardware interface for a robot
class BISONHWInterface : public hardware_interface::RobotHW
{
public:
  BISONHWInterface();

  /*
   *
   */
  void write()
  {
    double diff_ang_speed_left = cmd[0];
    double diff_ang_speed_right = cmd[1];
    // limitDifferentialSpeed(diff_ang_speed_left, diff_ang_speed_right);
    // Publish results
    std_msgs::Int32 left_wheel_vel_msg;
    std_msgs::Int32 right_wheel_vel_msg;
    bison_base::drive wheels_msg;
    left_wheel_vel_msg.data = int((diff_ang_speed_left / (2 * M_PI)) * 60);
    right_wheel_vel_msg.data = int((diff_ang_speed_right / (2 * M_PI)) * 60);
    left_wheel_vel_pub_.publish(left_wheel_vel_msg);
    right_wheel_vel_pub_.publish(right_wheel_vel_msg);
    //ROS_INFO_STREAM(left_wheel_vel_msg);
    wheels_msg.LEFT=left_wheel_vel_msg.data;
    wheels_msg.RIGHT=right_wheel_vel_msg.data ;

    wheels_vel_pub.publish(wheels_msg);
    ROS_INFO_STREAM(wheels_msg);
  }

  /**
   * Reading encoder values and setting position and velocity of enconders 
   */
  void read(const ros::Duration &period)
  {
    double ang_distance_left = _wheel_angle[0];
    double ang_distance_right = _wheel_angle[1];
    pos[0] += ang_distance_left;
    vel[0] += ang_distance_left / period.toSec();
    pos[1] += ang_distance_right;
    vel[1] += ang_distance_right / period.toSec();
    /*
    std::ostringstream os;
    for (unsigned int i = 0; i < NUM_JOINTS - 1; ++i)
    {
      os << cmd[i] << ", ";
      pos[i] = cmd[i];
    }
    os << cmd[NUM_JOINTS - 1];
    ROS_INFO_STREAM("Commands for joints: " << os.str());
    */
  }

  ros::Time get_time()
  {
    prev_update_time = curr_update_time;
    curr_update_time = ros::Time::now();
    return curr_update_time;
  }

  ros::Duration get_period()
  {
    return curr_update_time - prev_update_time;
  }

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[NUM_JOINTS];
  double pos[NUM_JOINTS];
  double vel[NUM_JOINTS];
  double eff[NUM_JOINTS];

  bool running_;
  double _wheel_diameter;
  double _max_speed;
  double _wheel_angle[NUM_JOINTS];

  ros::Time curr_update_time, prev_update_time;

  ros::Subscriber left_wheel_angle_sub_;
  ros::Subscriber right_wheel_angle_sub_;
  ros::Publisher left_wheel_vel_pub_;
  ros::Publisher right_wheel_vel_pub_;
  ros::Publisher wheels_vel_pub;
  ros::Subscriber wheels_angle_sub;

  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;

  bool start_callback(std_srvs::Empty::Request & /*req*/, std_srvs::Empty::Response & /*res*/)
  {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request & /*req*/, std_srvs::Empty::Response & /*res*/)
  {
    running_ = false;
    return true;
  }

  void leftWheelAngleCallback(const std_msgs::Float32 &msg)
  {
    _wheel_angle[0] = msg.data;
  }

  void rightWheelAngleCallback(const std_msgs::Float32 &msg)
  {
    _wheel_angle[1] = msg.data;
  }

  void limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
    double speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));
    if (speed > _max_speed)
    {
      diff_speed_left *= _max_speed / speed;
      diff_speed_right *= _max_speed / speed;
    }
  }

}; // class

BISONHWInterface::BISONHWInterface()
    : running_(true), private_nh("~"), start_srv_(nh.advertiseService("start", &BISONHWInterface::start_callback, this)), stop_srv_(nh.advertiseService("stop", &BISONHWInterface::stop_callback, this))
{
  private_nh.param<double>("wheel_diameter", _wheel_diameter, 0.15);
  private_nh.param<double>("max_speed", _max_speed, 1.0);

  // Intialize raw data
  std::fill_n(pos, NUM_JOINTS, 0.0);
  std::fill_n(vel, NUM_JOINTS, 0.0);
  std::fill_n(eff, NUM_JOINTS, 0.0);
  std::fill_n(cmd, NUM_JOINTS, 0.0);

  // connect and register the joint state and velocity interfaces
  hardware_interface::JointStateHandle state_handle_left("left_wheel_joint", &pos[0], &vel[0], &eff[0]);
  jnt_state_interface.registerHandle(state_handle_left);
  hardware_interface::JointStateHandle state_handle_right("right_wheel_joint", &pos[1], &vel[1], &eff[1]);
  jnt_state_interface.registerHandle(state_handle_right);

  hardware_interface::JointHandle vel_handle_left(jnt_state_interface.getHandle("left_wheel_joint"), &cmd[0]);
  jnt_vel_interface.registerHandle(vel_handle_left);
  hardware_interface::JointHandle vel_handle_right(jnt_state_interface.getHandle("right_wheel_joint"), &cmd[1]);
  jnt_vel_interface.registerHandle(vel_handle_right);
  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_vel_interface);

  // Initialize publishers and subscribers
  left_wheel_vel_pub_ = nh.advertise<std_msgs::Int32>("bison/left_wheel_vel", 1);
  right_wheel_vel_pub_ = nh.advertise<std_msgs::Int32>("bison/right_wheel_vel", 1);
  wheels_vel_pub= nh.advertise<bison_base::drive>("bison/wheels_vel", 1);

  left_wheel_angle_sub_ = nh.subscribe("bison/left_wheel_angle", 1, &BISONHWInterface::leftWheelAngleCallback, this);
  right_wheel_angle_sub_ = nh.subscribe("bison/right_wheel_angle", 1, &BISONHWInterface::rightWheelAngleCallback, this);
  wheels_angle_sub = nh.subscribe("bison/wheels_angle", 1, &BISONHWInterface::rightWheelAngleCallback, this);
}
