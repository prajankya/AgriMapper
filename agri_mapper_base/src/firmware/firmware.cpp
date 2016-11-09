#define READ_ODOM
#define READ_IMU
#define DEBUG

#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

void setup() {// ----------------------------------------- setup
        nh.initNode();

}

void loop() {
        delay(10);
}
