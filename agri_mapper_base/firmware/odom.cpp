#include "odom.h"
#include <Arduino.h>

void odom::init(ros::NodeHandle _nh, const char * topic, int e1A, int e1B, int e2A, int e2B){
        nh = _nh;

        ros::Publisher odom_pub_(topic, &odom_msg);
        odom_pub = &odom_pub_;
        nh.advertise(*odom_pub);
        left.init(e1A, e1B);
        right.init(e2A, e2B);

        oldL = 0;
        oldR = 0;
}

void odom::loop(){
        left.loop();
        right.loop();
        unsigned long nL = left.getPos();
        unsigned long nR = right.getPos();

        if(nL != oldL || nR != oldR) {
                char l[10];
                dtostrf(nL, 6, 2, l);

                char r[10];
                dtostrf(nR, 6, 2, r);

                String s = String(l) + "," + String(r);
                char bb[50];
                s.toCharArray(bb, 50);
                odom_msg.data = bb;
                odom_pub->publish(&odom_msg);
        }
}
