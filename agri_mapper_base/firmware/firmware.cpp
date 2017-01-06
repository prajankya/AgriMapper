//#define USE_ROS 1
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
#ifndef USE_ROS
  Serial.begin(9600);
#endif

#ifdef USE_ROS
  nh.initNode();
  nh.advertise(odom_pub);
  nh.advertise(imu_pub);
#endif

  odom.init(7, 8, 4, 6);

#ifdef USE_IMU
  imu.init();
#endif
}

unsigned long previousMillis = 0;

const long interval = 500;

void loop() {
  odom.loop();

#ifdef USE_IMU
  imu.loop();
#endif

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    //nh.loginfo(odom.msg);
#ifdef USE_ROS
    odom_msg.data = odom.msg;
    odom_pub.publish(&odom_msg);

  #ifdef USE_IMU
    imu_msg.data = imu.msg;
    imu_pub.publish(&imu_msg);
  #endif

#endif

#ifndef USE_ROS
    Serial.print("odom msg :");
    Serial.println(odom.msg);

  #ifdef USE_IMU
    Serial.print("\tIMU mag_msg :");
    Serial.println(imu.mag_msg);
    Serial.print("\tIMU acc_msg :");
    Serial.println(imu.acc_msg);
    Serial.print("\tIMU gyro_msg :");
    Serial.println(imu.gyro_msg);
    Serial.print("\tIMU baro_msg :");
    Serial.println(imu.baro_msg);
  #endif

#endif
  }

#ifdef USE_ROS
  nh.spinOnce();
#endif
}
