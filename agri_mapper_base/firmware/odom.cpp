#include "odom.h"
#include <Arduino.h>

void odom::init(ros::NodeHandle _nh, const char * topic){
        nh = _nh;

        ros::Publisher odom_pub_(topic, &odom_msg);
        odom_pub = &odom_pub_;
        nh.advertise(*odom_pub);
}
