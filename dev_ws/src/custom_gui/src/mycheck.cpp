#include "mycheck.h"

using namespace std;

MyCheck::MyCheck(QObject * parent)
      : QThread(parent)
    {
      ros::NodeHandle n;
      check_client_ = n.serviceClient<custom_gui::Check>("connect_robot");
    }

void MyCheck::run()
{
  ros::Time begin = ros::Time::now();
  if (!check_client_.call(srv_))
  {
    while (!check_client_.call(srv_) && (ros::Time::now() - begin).toSec() < 5.0)
    {
      ROS_ERROR("Failed to call service connect_robot");
      ros::Duration(1).sleep();
    }
    emit checkfail();
  }
  else if (srv_.response.success)
  {
    std::string message = srv_.response.message;
    ROS_INFO("%s", message.c_str());
    if (message == "opened")
    {
      emit checkcheck(0);
    }
    else if (message == "control" || message == "mapping" || message == "navigation")
    {
      emit checkcheck(1);
    }
    else if (message == "disconnected")
    {
      emit checkcheck(2);
    }
  }
  else
  {
    emit checkfail();
  }
}

void MyCheck::setData(string data)
{
  srv_.request.data = data;
}