#include <ros/ros.h>
#include <ros/console.h>

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>

void mapSubCallback(const nav_msgs::OccupancyGrid msg);

int main(int argc, char **argv) {
  ros::init(argc, argv, "tree_detector");

  ros::NodeHandle n;
  ros::Subscriber scan_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 50, mapSubCallback);

  while (n.ok()) {
    ros::spinOnce();
  }
}

void mapSubCallback(const nav_msgs::OccupancyGrid msg) {
  ROS_INFO_STREAM("Res: " << msg.info.resolution <<
                  "\t Width:" << msg.info.width <<
                  "\t Height:" << msg.info.height);
  return;
}