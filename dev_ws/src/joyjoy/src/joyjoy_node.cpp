#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <windows.h>
#include "joystickapi.h"
#include "mmsystem.h"

#pragma comment(lib, "winmm.lib")

double deadzone_ = 0.3;
double scale = -1. / (1. - deadzone_) / 32767.;
double unscaled_deadzone = 32767. * deadzone_;

double scale_val_linear(DWORD val)
{
    double ret = (val - 32767.) / 32767.;

    if (-0.1 < ret && ret < 0.1)
        return 0.;
    else if (ret < -1)
        return 0.15;
    else if (ret > 1)
        return -0.15;

    return ret * -0.15;
}

double scale_val_angular(DWORD val)
{
    double ret = (val - 32767.) / 32767.;

    if (-0.1 < ret && ret < 0.1)
        return 0.;
    else if (ret < -1)
        return 0.4;
    else if (ret > 1)
        return -0.4;

    return ret * -0.4;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joyjoy");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    JOYINFOEX joystickInfo;
    ZeroMemory(&joystickInfo, sizeof(joystickInfo));
    joystickInfo.dwSize = sizeof(joystickInfo);
    joystickInfo.dwFlags = JOY_RETURNALL;
    int button_old = 0;

    geometry_msgs::Twist msg;
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        joyGetPosEx(JOYSTICKID1, &joystickInfo);

        if (joystickInfo.dwButtons == 1)
        {
            double valY = scale_val_linear(joystickInfo.dwYpos);
            msg.linear.x = valY;
            double valX = scale_val_angular(joystickInfo.dwXpos);
            if (valY < 0)
                msg.angular.z = -valX;
            else
                msg.angular.z = valX;
            pub.publish(msg);
            button_old = 1;
        }
        else if (button_old == 1 && joystickInfo.dwButtons == 0)
        {
            msg.linear.x = 0;
            msg.angular.z = 0;
            pub.publish(msg);
            button_old = 0;
        }

        loop_rate.sleep();
    }
}