#define USE_ROS 1
#define USE_IMU 1

#ifdef USE_ROS
  #include <ros.h>
  #include <ros/time.h>
  #include <std_msgs/String.h>
#endif

#include <Arduino.h>
#include "odom.h"
#include "IMU.h"

#ifdef USE_ROS
ros::NodeHandle nh;

std_msgs::String odom_msg;
ros::Publisher odom_pub("odom_pub", &odom_msg);

  #ifdef USE_IMU
std_msgs::String imu_msg;
ros::Publisher imu_pub("imu_msg", &imu_msg);
  #endif

#endif

Odom odom;

#ifdef USE_IMU
IMU imu;
#endif

void setup() {
  pinMode(12, OUTPUT);
#ifndef USE_ROS
  Serial.begin(9600);
  Serial.println("Starting Node..");
#endif

#ifdef USE_ROS
  nh.initNode();

  nh.advertise(odom_pub);

  #ifdef USE_IMU
  nh.advertise(imu_pub);
  #endif
#endif // ifdef USE_ROS

  odom.init(A0, 10, 2, 4);

#ifdef USE_IMU
  #ifdef USE_ROS

  nh.loginfo("Starting IMU...");
  #endif // ifdef USE_ROS

  imu.init();

  #ifdef USE_ROS
  nh.loginfo("IMU started");
  #endif
#endif // ifdef USE_IMU
}

unsigned long previousMillis = 0;

const long interval = 1;

void loop() {
  odom.loop();

#ifdef USE_IMU
  imu.loop();
#endif

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

#ifdef USE_ROS
    odom_msg.data = odom.msg;
    odom_pub.publish(&odom_msg);

  #ifdef USE_IMU
    char imu_out[130];
    imu.toString().toCharArray(imu_out, 130);
    imu_msg.data = imu_out;

    imu_pub.publish(&imu_msg);
  #endif

#endif

#ifndef USE_ROS
    Serial.print("odom msg :");
    Serial.println(odom.msg);

  #ifdef USE_IMU
    Serial.print("\tIMU :");
    Serial.println(imu.toString());
  #endif

#endif
  }

#ifdef USE_ROS
  nh.spinOnce();
#endif
}
