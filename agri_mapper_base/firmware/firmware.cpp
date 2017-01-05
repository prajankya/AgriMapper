#define USE_ROS 1

#ifdef USE_ROS
  #include <ros.h>
  #include <ros/time.h>
  #include <std_msgs/String.h>
#endif

#include <Arduino.h>
#include "odom.h"

#ifdef USE_ROS
ros::NodeHandle nh;

std_msgs::String odom_msg;
ros::Publisher odom_pub("odom_pub", &odom_msg);
#endif

Odom odom;

void setup() {
#ifdef USE_ROS
  nh.initNode();
  nh.advertise(odom_pub);
#endif
  odom.init(7, 8, 4, 6);
}

unsigned long previousMillis = 0;

const long interval = 50;

void loop() {
  odom.loop();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    //nh.loginfo(odom.msg);
#ifdef USE_ROS
    odom_msg.data = odom.msg;
    odom_pub.publish(&odom_msg);
#endif
  }

#ifdef USE_ROS
  nh.spinOnce();
#endif
}
