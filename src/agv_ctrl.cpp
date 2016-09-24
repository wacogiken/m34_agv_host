#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <m34_agv_host/agv_hard.h>
#include <m34_agv_host/agv_msgs.h>

class Abh3Agv
{
private:
    TAgv agv;
    controller_manager::ControllerManager *cm;

    ros::NodeHandle nh;
    ros::Subscriber abh3_diag;

public:
    Abh3Agv()
    {
        cm = new controller_manager::ControllerManager(&agv, nh);
        ros::NodeHandle abh3_node;
        abh3_diag  = abh3_node.subscribe("agv",  1, &Abh3Agv::abh3DiagCallback,  this);
    }

    void abh3DiagCallback(const m34_agv_host::agv_msgs &abh3_diag)
    {
        ros::Time now = agv.getTime();
        ros::Duration dt = agv.getPeriod();

        agv.read(now, dt, abh3_diag);
        cm->update(now, dt);

        agv.write(now, dt);
    }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "agv_ctrl");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Abh3Agv agv_;

  ros::spin();

  spinner.stop();

  return 0;
}

