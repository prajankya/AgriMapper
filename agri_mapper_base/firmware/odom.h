#ifndef __ODOM_H__
#define __ODOM_H__

#include<Arduino.h>
#include<ros.h>
#include<ros/time.h>
#include<std_msgs/String.h>
#include "encoder.h"

class odom {
  private:
    ros::NodeHandle nh;
    ros::Publisher * odom_pub;
    std_msgs::String odom_msg;
    Encoder left;
    Encoder right;
    unsigned long oldL;
    unsigned long oldR;

  public:
    void init(ros::NodeHandle _nh, const char * topic, int e1A, int e1B, int e2A, int e2B);
    void loop();
};
#endif
