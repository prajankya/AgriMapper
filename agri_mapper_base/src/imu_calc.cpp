#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

#include <iomanip>
#include <iostream>
#include <sstream>

#define PI 3.1415926535897931

ros::Time current_time, last_time;
ros::Publisher imu_pub;

double string_to_double(const std::string& s) {
  std::istringstream i(s);
  double x;

  if (!(i >> x)) return 0;

  return x;
}

void imuCallback(const std_msgs::String::ConstPtr & msg) {
  current_time = ros::Time::now();

  std::istringstream ss(msg->data);
  std::string token;
  int i = 0;
  std::string in[2]; //number of comma separated values

  while (std::getline(ss, token, ',')) {
    in[i++] = token;
  }

  //enL = round(string_to_double(in[0]));
  ROS_INFO_STREAM("got imu data");

  last_time = current_time;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_calc");

  ros::NodeHandle n;
  ros::Subscriber imu_sub = n.subscribe<std_msgs::String>("imu_msg", 50, imuCallback);

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate rate(10);

  while (n.ok()) {
    ros::spinOnce();

    rate.sleep();
  }
}
