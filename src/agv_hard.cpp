#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <m34_agv_host/agv_hard.h>
#include <m34_agv_host/agv_msgs.h>
#include <iostream> // for debug
#include <math.h>

TAgv::TAgv()
{
  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_1("base_l_wheel_joint", &pos_[0], &vel_[0], &eff_[0]);
  jnt_state_interface.registerHandle(state_handle_1);
  hardware_interface::JointStateHandle state_handle_2("base_r_wheel_joint", &pos_[1], &vel_[1], &eff_[1]);
  jnt_state_interface.registerHandle(state_handle_2);
  hardware_interface::JointStateHandle state_handle_3("table_joint", &pos_[2], &vel_[2], &eff_[2]);
  jnt_state_interface.registerHandle(state_handle_3);
  registerInterface(&jnt_state_interface);

  // connect and register the joint velocity interface
  hardware_interface::JointHandle vel_handle_1(jnt_state_interface.getHandle("base_l_wheel_joint"), &cmd_[0]);
  jnt_vel_interface.registerHandle(vel_handle_1);
  hardware_interface::JointHandle vel_handle_2(jnt_state_interface.getHandle("base_r_wheel_joint"), &cmd_[1]);
  jnt_vel_interface.registerHandle(vel_handle_2);
  hardware_interface::JointHandle vel_handle_3(jnt_state_interface.getHandle("table_joint"), &cmd_[2]);
  jnt_vel_interface.registerHandle(vel_handle_3);
  registerInterface(&jnt_vel_interface);
}

#define PI (3.1415926)
#define RATIO (20.0)

void TAgv::read(ros::Time time, ros::Duration period, m34_agv_host::agv_msgs abh3_diag)
{
    float velX, velY, velA, velB, a, b, c;

    velY = abh3_diag.velAY * 2.0 * PI / 60.0;
    velX = abh3_diag.velBX * 2.0 * PI / 60.0;
    velA = (velY + velX) / RATIO;
    velB = (velY - velX) / RATIO;

    eff_[0] = 0.0;
    vel_[0] = velA;
    pos_[0]+= velA * period.toSec();

    eff_[1] = 0.0;
    vel_[1] = velB;
    pos_[1]+= velB * period.toSec();

    eff_[2] = 0.0;
    vel_[2] = 0.0;
    if (abh3_diag.adrs==4 || abh3_diag.adrs==10) {
        pos_[2] += (velA - velB) * period.toSec() / (2.0 * PI);
    }
    else {
        pos_[2] = float(abh3_diag.pote)/10.0 * 3.1415926 / 180.0;
    }
}

void TAgv::write(ros::Time time, ros::Duration period)
{
}

