#ifndef __ODOM_H__
#define __ODOM_H__

#include<Arduino.h>
#include<ros.h>
#include<ros/time.h>
#include<std_msgs/String.h>

class odom {
  private:
    ros::NodeHandle nh;
    ros::Publisher * odom_pub;
    std_msgs::String odom_msg;

  public:
    void init(ros::NodeHandle _nh, const char * topic);
    void loop();
};
#endif
