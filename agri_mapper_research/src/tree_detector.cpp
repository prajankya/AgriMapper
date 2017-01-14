#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "tree_detector");

  ros::NodeHandle n;

  while (n.ok()) {
    ros::spinOnce();
  }
}
