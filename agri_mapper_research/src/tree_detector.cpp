#include <ros/ros.h>
#include <ros/console.h>

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
  return;
}