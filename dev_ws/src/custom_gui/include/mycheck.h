#ifndef MY_CHECK_H
#define MY_CHECK_H

#include <cstdio>
#include "ros/ros.h"
#include <custom_gui/Check.h>
#include <QThread>

using namespace std;

/**
 * @brief Check subscriber node.
 */
class MyCheck : public QThread
{
Q_OBJECT
  public:
    MyCheck(QObject * parent = 0);
    void setData(string data);

  private:
    ros::ServiceClient check_client_;
    custom_gui::Check srv_;

  protected:
    void run() override;

  signals:
    void checkcheck(int d);
    void checkfail();
};

#endif // MY_CHECK_H