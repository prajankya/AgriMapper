#include <ros.h>
#include <Arduino.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include "odom.h"

ros::NodeHandle nh;

std_msgs::String odom_msg;
ros::Publisher odom_pub("odom_pub", &odom_msg);

Odom odom;

void setup(){
        nh.initNode();
        nh.advertise(odom_pub);
        odom.init(7, 8, 4, 6);
}


unsigned long previousMillis = 0;

const long interval = 50;

void loop(){
        odom.loop();

        unsigned long currentMillis = millis();

        if (currentMillis - previousMillis >= interval) {
                previousMillis = currentMillis;

                //nh.loginfo(odom.msg);
                odom_msg.data = odom.msg;
                odom_pub.publish(&odom_msg);
        }
        nh.spinOnce();
}
