#include "mymap.h"

using namespace std;

MapGenerator::MapGenerator(const std::string& mapname, QObject * parent)
      : path_("c:/dev_ws/map/"), QThread(parent), mapname_(mapname), saved_map_(false), threshold_occupied_(65), threshold_free_(25)
    {
      ros::NodeHandle n;
      ROS_INFO("Waiting for the map");
      map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
      loop_rate = new ros::Rate(20);
      error_duration = new ros::Duration(1);
    }

void MapGenerator::mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
{
    ROS_INFO("Received a %d X %d map @ %.3f m/pix",
            map->info.width,
            map->info.height,
            map->info.resolution);


    std::string mapdatafile = path_ + mapname_ + ".pgm";
    ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
    FILE* out = fopen(mapdatafile.c_str(), "w");
    if (!out)
    {
    ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
    return;
    }

    fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
            map->info.resolution, map->info.width, map->info.height);
    for(unsigned int y = 0; y < map->info.height; y++) {
    for(unsigned int x = 0; x < map->info.width; x++) {
        unsigned int i = x + (map->info.height - y - 1) * map->info.width;
        if (map->data[i] >= 0 && map->data[i] <= threshold_free_) { // [0,free)
        fputc(254, out);
        } else if (map->data[i] >= threshold_occupied_) { // (occ,255]
        fputc(000, out);
        } else { //occ [0.25,0.65]
        fputc(205, out);
        }
    }
    }

    fclose(out);


    std::string mapmetadatafile = path_ + mapname_ + ".yaml";
    ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
    FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


      /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

       */

    geometry_msgs::Quaternion orientation = map->info.origin.orientation;
    tf2::Matrix3x3 mat(tf2::Quaternion(
    orientation.x,
    orientation.y,
    orientation.z,
    orientation.w
    ));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    std::string pgm_name = mapname_ + ".pgm";
    fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
            pgm_name.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

    fclose(yaml);

    ROS_INFO("Done\n");
    saved_map_ = true;
}

void MapGenerator::run()
{
  while(!saved_map_ && ros::ok())
  {
    ros::spinOnce();
  }
  emit saveMap();
}

