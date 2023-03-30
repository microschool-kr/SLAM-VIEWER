#ifndef MY_MAP_H
#define MY_MAP_H

#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include <QThread>
#include <tf/transform_listener.h>

using namespace std;

/**
 * @brief Map generation node.
 */
class MapGenerator : public QThread
{
Q_OBJECT
  public:
    MapGenerator(const std::string& mapname, QObject * parent = 0);
    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map);
    void run();

  private:
    std::string path_;
    std::string mapname_;
    ros::Subscriber map_sub_;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Rate * loop_rate;
    ros::Duration * error_duration;
    bool saved_map_;
    int threshold_occupied_;
    int threshold_free_;
  
  signals:
    void saveMap();
};

#endif // MY_MAP_H