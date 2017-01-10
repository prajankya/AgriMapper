#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <iomanip>
#include <iostream>
#include <sstream>

#define PI 3.1415926535897931

int64_t enL = 0;
int64_t enR = 0;

int64_t oldenL = 0;
int64_t oldenR = 0;

double wheelCircumference = 0.51;//meters
double wheelDistance = 0.54;//meters

uint8_t encoderResolution = 16;

ros::Time current_time, last_time;
ros::Publisher odom_pub;

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

double string_to_double(const std::string& s) {
  std::istringstream i(s);
  double x;

  if (!(i >> x)) return 0;

  return x;
}

void odomCallback(const std_msgs::String::ConstPtr & msg) {
  current_time = ros::Time::now();

  std::istringstream ss(msg->data);
  std::string token;
  int i = 0;
  std::string in[2];

  while (std::getline(ss, token, ',')) {
    in[i++] = token;
  }

  enL = round(string_to_double(in[0]));
  enR = round(string_to_double(in[1]));

  //ROS_INFO_STREAM("Left:" << enL << "\tRight:" << enR);

  //-------------------- Convert enL and enR into Meters -------------
  int difL = 0, difR = 0;

  if (enL != oldenL) {
    difL = enL - oldenL;
    oldenL = enL;
  }

  if (enR != oldenR) {
    difR = enR - oldenR;
    oldenR = enR;
  }

  double dl = (difL * wheelCircumference) / encoderResolution;
  double dr = (difR * wheelCircumference) / encoderResolution;

  //ROS_INFO_STREAM("Left:" << dl << "\tRight:" << dr);

  //=============================== Calculate x, y, th ====================
  double dth = asin((dr - dl) / wheelDistance);

  double l = wheelDistance / 2;

  double dx = l * sin(dth);
  double dy = l - (l * cos(dth));

  double dt = (current_time - last_time).toSec();
  vx = dx / dt;
  vy = dy / dt;
  vth = dth / dt;

  x += dx;
  y += dy;
  th += dth;

  if (th > 2 * PI) {
    th = 0;
  } else if (th < 2 * PI) {
    th = 0;
  }

  last_time = current_time;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_calc");

  ros::NodeHandle n;
  ros::Subscriber odom_sub = n.subscribe<std_msgs::String>("odom_pub", 50, odomCallback);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate rate(10);

  while (n.ok()) {
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    tf::TransformBroadcaster odom_broadcaster;
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    odom_pub.publish(odom);
    ros::spinOnce();

    rate.sleep();
  }
}
