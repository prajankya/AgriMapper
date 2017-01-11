#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

#include <iomanip>
#include <iostream>
#include <sstream>

#define PI 3.1415926535897931

ros::Time current_time, last_time;
ros::Publisher imu_pub;
sensor_msgs::Imu imu_msg;
double string_to_double(const std::string& s) {
  std::istringstream i(s);
  double x;

  if (!(i >> x)) return 0;

  return x;
}

using namespace std;

void tokenize(const string& str, vector<string>& tokens, const string& delimiters = ",") {
  // Skip delimiters at beginning.
  string::size_type lastPos = str.find_first_not_of(delimiters, 0);

  // Find first non-delimiter.
  string::size_type pos = str.find_first_of(delimiters, lastPos);

  while (string::npos != pos || string::npos != lastPos) {
    // Found a token, add it to the vector.
    tokens.push_back(str.substr(lastPos, pos - lastPos));

    // Skip delimiters.
    lastPos = str.find_first_not_of(delimiters, pos);

    // Find next non-delimiter.
    pos = str.find_first_of(delimiters, lastPos);
  }
}

void imuCallback(const std_msgs::String::ConstPtr & msg) {
  current_time = ros::Time::now();

  //ROS_INFO_STREAM("got imu data : " << msg->data);

  std::istringstream ss(msg->data);
  std::string token;
  int i = 0;

  std::string sensors[4]; //number of colon separated values

  while (std::getline(ss, token, ':')) {
    sensors[i++] = token;
  }

  double mag[3], acc[3], gyro[6], baro[2];

  vector<string> mag_tokens;
  tokenize(sensors[0], mag_tokens);

  for (i = 0; i < mag_tokens.size(); i++) {
    mag[i] = string_to_double(mag_tokens.at(i));
  }

  vector<string> acc_tokens;
  tokenize(sensors[1], acc_tokens);

  for (i = 0; i < acc_tokens.size(); i++) {
    acc[i] = string_to_double(acc_tokens.at(i));
  }

  vector<string> gyro_tokens;
  tokenize(sensors[2], gyro_tokens);

  for (i = 0; i < gyro_tokens.size(); i++) {
    gyro[i] = string_to_double(gyro_tokens.at(i));
  }

  vector<string> baro_tokens;
  tokenize(sensors[3], baro_tokens);

  for (i = 0; i < baro_tokens.size(); i++) {
    baro[i] = string_to_double(baro_tokens.at(i));
  }

//  ROS_INFO_STREAM("ACC:" << acc[0] << "," << acc[1] << "," << acc[2]);

  imu_msg.linear_acceleration.x = acc[0];
  imu_msg.linear_acceleration.y = acc[1];
  imu_msg.linear_acceleration.z = acc[2];

  imu_msg.angular_velocity.x = gyro[3];
  imu_msg.angular_velocity.y = gyro[4];
  imu_msg.angular_velocity.z = gyro[5];


  imu_msg.orientation.w = 0.0;
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;

  last_time = current_time;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_calc");

  ros::NodeHandle n;
  ros::Subscriber imu_sub = n.subscribe<std_msgs::String>("imu_msg", 50, imuCallback);
  imu_pub = n.advertise<sensor_msgs::Imu>("imu_raw", 50);

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate rate(10);

  while (n.ok()) {
    ros::spinOnce();

    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "chassis_link";

    imu_pub.publish(imu_msg);

    rate.sleep();
  }
}
