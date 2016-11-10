#define READ_ODOM
#define READ_IMU
#define DEBUG

#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>

#ifdef READ_IMU
  #include <IMU.h>
#endif

#ifdef READ_ODOM
  #include <odom.h>
#endif

ros::NodeHandle nh;

void setup() {// ----------------------------------------- setup
        nh.initNode();

}

void loop() {
        delay(10);
}
