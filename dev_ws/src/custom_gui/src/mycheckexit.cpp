#include "mycheckexit.h"

using namespace std;

MyCheckExit::MyCheckExit(QObject * parent)
      : QThread(parent)
    {
      ros::NodeHandle n;
      exit_server_ = n.advertiseService("connect_lost", &MyCheckExit::checkCallback, this);
    }

bool MyCheckExit::checkCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  emit checkexit();
  return true;
}

void MyCheckExit::run()
{
  ROS_INFO("run");
  ros::spin();
}