#include <ros/ros.h>
#include <ros/console.h>

#include <dynamic_reconfigure/server.h>
#include "agri_mapper_research/dynReConfig.h"

#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include "opencv2/opencv.hpp"

#include <string>

//dynamic params
int GaussianBlur_kernelSize;

float minThreshold;
float maxThreshold;

float minConvexity, maxConvexity;
float minArea, maxArea;
float minCircularity, maxCircularity;

void DynamicParamCallback(agri_mapper_research::dynReConfig &config, uint32_t level);
void mapSubCallback(const nav_msgs::OccupancyGrid map);
// void printKeypoints(std::vector<cv::KeyPoint> keypoints);
// void detectTrees();
// void mapToImage();

cv_bridge::CvImage cv_img, cv_detectionImg;
ros::Publisher image_pub, detected_pub, vis_pub;
//nav_msgs::OccupancyGrid map;

double MAP_RESOLUTION;
long MAP_X, MAP_Y;

int main(int argc, char **argv) {
  ros::init(argc, argv, "tree_detector");

  ros::NodeHandle n("~");

  dynamic_reconfigure::Server<agri_mapper_research::dynReConfig> server;
  dynamic_reconfigure::Server<agri_mapper_research::dynReConfig>::CallbackType f;

  f = boost::bind(&DynamicParamCallback, _1, _2);
  server.setCallback(f);

  ros::Subscriber scan_sub = n.subscribe("/map", 50, mapSubCallback);

  image_pub = n.advertise<sensor_msgs::Image>("image", 50);
  detected_pub = n.advertise<sensor_msgs::Image>("detection_image", 50);
  vis_pub = n.advertise<visualization_msgs::Marker>("detected_trees", 0);

  cv_img.header.frame_id = "map_image";
  cv_img.encoding = sensor_msgs::image_encodings::MONO8;

  cv_detectionImg.header.frame_id = "map_detectedCluster";
  cv_detectionImg.encoding = sensor_msgs::image_encodings::RGB8;

  GaussianBlur_kernelSize  = 3;

  minThreshold = 0;
  maxThreshold = 400;
  minConvexity = 0;
  maxConvexity = 0;
  minCircularity = 0;
  maxCircularity = 0;
  minArea = 55;
  maxArea = 200;

  ros::Rate rate(10);

  while (n.ok()) {
    ros::spinOnce();

    // if (map.header.frame_id != "") {
    //   detectTrees();
    // }

    rate.sleep();
  }
}

void DynamicParamCallback(agri_mapper_research::dynReConfig &config, uint32_t level) {
  ROS_INFO("Reconfigured parameters");

  GaussianBlur_kernelSize  = config.GaussianBlur_kernelSize;

  minThreshold = config.minThreshold;
  maxThreshold = config.maxThreshold;
  minConvexity = config.minConvexity;
  maxConvexity = config.maxConvexity;
  minCircularity = config.minCircularity;
  maxCircularity = config.maxCircularity;
  minArea = config.minArea;
  maxArea = config.maxArea;
}

void mapSubCallback(const nav_msgs::OccupancyGrid map) {
  //================================================================MAP TO IMAGE
  MAP_X = map.info.width;
  MAP_Y = map.info.height;

  MAP_RESOLUTION = map.info.resolution;

  if ((MAP_X < 3) || (MAP_Y < 3) ) {
    ROS_INFO("Map size is only x: %ld,  y: %ld . Not running map to image conversion", MAP_X, MAP_Y);
    return;
  }

  cv::Mat *map_mat  = &cv_img.image;

  // resize cv image if it doesn't have the same dimensions as the map
  if ((map_mat->rows != MAP_Y) && (map_mat->cols != MAP_X)) {
    *map_mat = cv::Mat(MAP_Y, MAP_X, CV_8U);
  }

  // ROS_INFO("Fault_1");

  const std::vector<int8_t>&map_data(map.data);

  unsigned char *map_mat_data_p = (unsigned char *)map_mat->data;

  // ROS_INFO("Fault_2");

  //We have to flip around the y axis, y for image starts at the top and y for map at the bottom

  int MAP_Y_rev = MAP_Y - 1;

  // ROS_INFO_STREAM("Fault for :" << MAP_Y_rev);

  for (int y = MAP_Y_rev; y >= 0; --y) {
    int idx_map_y = MAP_X * (MAP_Y - y);
    int idx_img_y = MAP_X * y;

    // ROS_INFO_STREAM("Fault_3 :" << y);

    // if (y == 0) {
    //   ROS_INFO_STREAM("MAP_X :" << MAP_X);
    // }

    for (int x = 0; x < MAP_X; ++x) {
      int idx = idx_img_y + x;
      //
      // if (y == 0 && x > 1000) {
      //   ROS_INFO_STREAM("x :" << x);
      //   try {
      //     //ROS_INFO_STREAM("valid:" << isValidPtr(map_data, idx_map_y + x));
      //     ROS_INFO_STREAM("Map data :" << map_data[idx_map_y + x]);
      //   } catch (std::exception& e) {
      //     std::cout << e.what() << std::endl;
      //   }
      // }

      switch (map_data[idx_map_y + x]) {
        case -1:
          map_mat_data_p[idx] = 255; //grey color:unknown default value of 127
          break;

        case 0:
          map_mat_data_p[idx] = 255; // laser ray:default 255
          break;

        case 100:
          map_mat_data_p[idx] = 0; // obstacle:default 0
          break;
      }

      // if (y == 0 && x > 1000) {
      //   ROS_INFO_STREAM("x :" << x);
      // }
    }

    // ROS_INFO_STREAM("Fault_3 :" << y);
  }

  // ROS_INFO("Fault_4");


//======================================================================== DETECT TREES
  cv::Mat im, im_with_keypoints;

// Reduce the noise
  if (GaussianBlur_kernelSize > 1) {
    cv::GaussianBlur(cv_img.image, im, cv::Size(GaussianBlur_kernelSize, GaussianBlur_kernelSize), 0);
  } else {
    im = cv_img.image;
  }

// Setup SimpleBlobDetector parameters.
  cv::SimpleBlobDetector::Params params;

// Change thresholds
  params.minThreshold = minThreshold;
  params.maxThreshold = maxThreshold;

// Filter by Area.
  if (minArea < maxArea) {
    params.filterByArea = true;
    params.minArea = minArea;
    params.maxArea = maxArea;
  } else {
    params.filterByArea = false;
  }

// Filter by Circularity
  if (minCircularity < maxCircularity) {
    params.filterByCircularity = true;
    params.minCircularity = minCircularity;
    params.maxCircularity = maxCircularity;
  } else {
    params.filterByCircularity = false;
  }

// Filter by Convexity
  if (minConvexity < maxConvexity) {
    params.filterByConvexity = true;
    params.minConvexity = minConvexity;
    params.maxConvexity = maxConvexity;
  } else {
    params.filterByConvexity = false;
  }

// Filter by Inertia
// params.filterByInertia = true;
// params.minInertiaRatio = 0.01;

// Storage for blobs
  std::vector<cv::KeyPoint> keypoints;

// Set up detector with params
  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

// Detect blobs
  detector->detect(im, keypoints);

//================================================================ Publish Markers
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "agri_mapper";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = -(MAP_X * MAP_RESOLUTION / 2) - 0.2;
  marker.pose.position.y = -(MAP_Y * MAP_RESOLUTION / 2) - 0.2;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker.points.resize(keypoints.size());

  std::vector<cv::KeyPoint>::iterator it = keypoints.begin();

  ROS_INFO_STREAM("making markers");
  ROS_INFO_STREAM("\n Size : " << keypoints.size() << "\n");

  for (int i = 0; it != keypoints.end(); ++it, i++) {
    marker.points[i].x = (it->pt.x) * MAP_RESOLUTION;
    marker.points[i].y = (MAP_Y - it->pt.y) * MAP_RESOLUTION;
    marker.points[i].z = 0;
  }

  ROS_INFO_STREAM("made markers");

  vis_pub.publish(marker);

//============================================================================PUBLISH IMAGE

// Draw detected blobs as red circles.
// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
// the size of the circle corresponds to the size of blob

  cv::drawKeypoints(im, keypoints, im_with_keypoints, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  cv_detectionImg.image = im_with_keypoints;

  image_pub.publish(cv_img.toImageMsg());
  detected_pub.publish(cv_detectionImg.toImageMsg());
}
