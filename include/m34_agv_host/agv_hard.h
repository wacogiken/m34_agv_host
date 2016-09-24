#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <m34_agv_host/agv_msgs.h>
#include <map>
#include <string>
#include <vector>

class TAgv : public hardware_interface::RobotHW
{
public:
  TAgv();

  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.05); }

  void read(ros::Time, ros::Duration, m34_agv_host::agv_msgs);
  void write(ros::Time, ros::Duration);

protected:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd_[3];
  double pos_[3];
  double vel_[3];
  double eff_[3];

};

