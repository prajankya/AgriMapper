#include <ros/ros.h>
#include <ros/console.h>

#define PI 3.1415926535897931

int main(int argc, char** argv){
        ros::init(argc, argv, "odom_calc");

        ros::NodeHandle n;

        while(n.ok()) {
                ros::spinOnce();
        }
}
