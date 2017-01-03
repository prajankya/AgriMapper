#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>

#define PI 3.1415926535897931

sensor_msgs::LaserScan outScan;

double clip_1 = 90.0;
double clip_2 = 270.0;
int num_readings = 0;
ros::Publisher scan_pub;

void scanSubCallback(const sensor_msgs::LaserScan msg){

  outScan.header.stamp = ros::Time::now();
  if(outScan.angle_increment == 0) {
    outScan.angle_increment = msg.angle_increment;
    outScan.time_increment = msg.time_increment;
    outScan.range_min = msg.range_min;
    outScan.range_max = msg.range_max;
  }

  for(int i = clip_1; i < clip_2; i++) {
    outScan.ranges[i - clip_1] = msg.ranges[i];
    outScan.intensities[i - clip_1] = msg.intensities[i];
  }

  scan_pub.publish(outScan);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "scan_clipper");

  ros::NodeHandle n;
  ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("scan", 50, scanSubCallback);
  scan_pub = n.advertise<sensor_msgs::LaserScan>("clipped_scan", 50);

  n.param("scan_clipper/min_angle", clip_1, 90.0);
  n.param("scan_clipper/max_angle", clip_2, 270.0);

  ROS_INFO_STREAM("min_angle : " << clip_1);
  ROS_INFO_STREAM("max_angle : " << clip_2);


  outScan.header.frame_id = "rplidar_frame";
  outScan.angle_min = ((clip_1 - 180) / 180) * PI;
  outScan.angle_max = ((clip_2 - 180) / 180) * PI;

  num_readings = clip_2 - clip_1;

  outScan.angle_increment = 0;
  outScan.time_increment = 0;

  outScan.ranges.resize(num_readings);
  outScan.intensities.resize(num_readings);

  while(n.ok()) {
    ros::spinOnce();
  }
}
