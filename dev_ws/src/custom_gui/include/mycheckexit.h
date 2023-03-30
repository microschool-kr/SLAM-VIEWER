#ifndef MY_CHECK_EXIT_H
#define MY_CHECK_EXIT_H

#include <cstdio>
#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <QThread>

using namespace std;

/**
 * @brief Check subscriber node.
 */
class MyCheckExit : public QThread
{
Q_OBJECT
  public:
    MyCheckExit(QObject * parent = 0);
    void run();
    bool checkCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  private:
    ros::ServiceServer exit_server_;
  
  signals:
    void checkexit();
};

#endif // MY_CHECK_EXIT_H