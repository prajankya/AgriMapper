#include <ros/ros.h>
#include <ros/console.h>

#include <dynamic_reconfigure/server.h>
#include "agri_mapper_research/dynReConfig.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>
#include "opencv2/opencv.hpp"

//dynamic params
int GaussianBlur_kernelSize;

float minThreshold;
float maxThreshold;

float minConvexity, maxConvexity;

void DynamicParamCallback(agri_mapper_research::dynReConfig &config, uint32_t level);
void mapSubCallback(const nav_msgs::OccupancyGridConstPtr& map);
void detectTrees();
void mapToImage();

cv_bridge::CvImage cv_img, cv_detectionImg;
ros::Publisher image_pub, detected_pub;

nav_msgs::OccupancyGrid map;

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

  cv_img.header.frame_id = "map_image";
  cv_img.encoding = sensor_msgs::image_encodings::MONO8;

  cv_detectionImg.header.frame_id = "map_detectedCluster";
  cv_detectionImg.encoding = sensor_msgs::image_encodings::MONO16;

  ros::Rate rate(10);

  while (n.ok()) {
    ros::spinOnce();

    if (map.header.frame_id != "") {
      detectTrees();
    }

    rate.sleep();
  }
}

void DynamicParamCallback(agri_mapper_research::dynReConfig &config, uint32_t level) {
  ROS_INFO("Reconfigured parameters");

  GaussianBlur_kernelSize  = config.GaussianBlur_kernelSize;

  minThreshold = config.minThreshold;
  maxThreshold = config.maxThreshold;
  minConvexity = config.minConvexity;
}

void mapSubCallback(const nav_msgs::OccupancyGridConstPtr& map_) {
  map = *map_;
  detectTrees();
}

void detectTrees() {
  mapToImage();
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
  //params.filterByArea = true;
  //params.minArea = 1500;

  // Filter by Circularity
  //params.filterByCircularity = true;
  //params.minCircularity = 0.8;

  // Filter by Convexity
  if (minConvexity != 0.0) {
    params.filterByConvexity = true;
    params.minConvexity = minConvexity;
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

  // Draw detected blobs as red circles.
  // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
  // the size of the circle corresponds to the size of blob

  cv::drawKeypoints(im, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  //cv_detectionImg.image = im_with_keypoints;
  // Show blobs
//  imshow("keypoints", im_with_keypoints);
  cv_detectionImg.image = im_with_keypoints;
  /*
     Canny(cv_detectionImg.image, cv_detectionImg.image, 500, 200);//just applied canny for visibility
   */

  image_pub.publish(cv_img.toImageMsg());
  detected_pub.publish(cv_detectionImg.toImageMsg());
}

void mapToImage() {
  int size_x = map.info.width;
  int size_y = map.info.height;

  if ((size_x < 3) || (size_y < 3) ) {
    ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
    return;
  }

  cv::Mat *map_mat  = &cv_img.image;

  // resize cv image if it doesn't have the same dimensions as the map
  if ( (map_mat->rows != size_y) && (map_mat->cols != size_x)) {
    *map_mat = cv::Mat(size_y, size_x, CV_8U);
  }

  if ((cv_detectionImg.image.rows != size_y) && (cv_detectionImg.image.cols != size_x)) {
    cv_detectionImg.image = cv::Mat(size_y, size_x, CV_8U, 255);
  }

  const std::vector<int8_t>& map_data(map.data);

  unsigned char *map_mat_data_p = (unsigned char *)map_mat->data;

  //We have to flip around the y axis, y for image starts at the top and y for map at the bottom

  int size_y_rev = size_y - 1;

  for (int y = size_y_rev; y >= 0; --y) {
    int idx_map_y = size_x * (size_y - y);
    int idx_img_y = size_x * y;

    for (int x = 0; x < size_x; ++x) {
      int idx = idx_img_y + x;

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
    }
  }

  return;
}
