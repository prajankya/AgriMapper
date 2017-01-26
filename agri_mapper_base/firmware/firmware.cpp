#define USE_ROS 1
#define USE_IMU 1
#define USE_ODOM 1

#include <Arduino.h>

#ifdef USE_ROS
  #include <ros.h>
  #include <ros/time.h>
  #include <std_msgs/String.h>
#endif

#ifdef USE_ODOM
//#define ENCODER_OPTIMIZE_INTERRUPTS
#include "Encoder/Encoder.h"
Encoder left(2, 10);
Encoder right(3, 11);
char odom_str[50];
#endif

#ifdef USE_IMU
#include "IMU.h"
IMU imu;
#endif

#ifdef USE_ROS
ros::NodeHandle nh;

  #ifdef USE_ODOM
    std_msgs::String odom_msg;
    ros::Publisher odom_pub("odom_pub", &odom_msg);
  #endif

  #ifdef USE_IMU
  std_msgs::String imu_msg;
  ros::Publisher imu_pub("imu_msg", &imu_msg);
  #endif

#endif

void setup() {
  pinMode(12, OUTPUT);
#ifndef USE_ROS
  Serial.begin(9600);
  Serial.println("Starting Node..");
#endif

#ifdef USE_ROS
  nh.initNode();

  #ifdef USE_ODOM
  nh.advertise(odom_pub);
  #endif

  #ifdef USE_IMU
  nh.advertise(imu_pub);
  #endif
#endif // ifdef USE_ROS

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
#ifdef USE_IMU
  imu.loop();
#endif

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;


    #ifdef USE_ODOM
      long leftEn = left.read();
      long rightEn = left.read();

      char l[10];
      dtostrf(leftEn, 6, 2, l);

      char r[10];
      dtostrf(rightEn, 6, 2, r);

      String s = String(l) + "," + String(r);
      s.toCharArray(odom_str, 50);
    #endif

#ifdef USE_ROS

  #ifdef USE_ODOM
    odom_msg.data = odom_str;
    odom_pub.publish(&odom_msg);
  #endif

  #ifdef USE_IMU
    char imu_out[130];
    imu.toString().toCharArray(imu_out, 130);
    imu_msg.data = imu_out;

    imu_pub.publish(&imu_msg);
  #endif

#endif

#ifndef USE_ROS
  #ifdef USE_ODOM
    Serial.print("odom msg :");
    Serial.println(odom_str);
  #endif

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
