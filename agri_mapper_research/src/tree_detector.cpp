#include <ros/ros.h>
#include <ros/console.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>

void mapSubCallback(const nav_msgs::OccupancyGridConstPtr& map);

cv_bridge::CvImage cv_img, cv_circlesImg;
ros::Publisher image_pub, circles_pub;


//---params
double inverse_ratio_of_resolution;
double minimum_distance_between_detected_centers;
double upper_threshold_for_canny_detector;
double center_detection_threshold;
int min_radius, max_radius;

int main(int argc, char **argv) {
  ros::init(argc, argv, "tree_detector");

  ros::NodeHandle n;

  n.param("inverse_ratio_of_resolution", inverse_ratio_of_resolution, 1.0);
  n.param("minimum_distance_between_detected_centers", minimum_distance_between_detected_centers, 1.0);
  n.param("upper_threshold_for_canny_detector", upper_threshold_for_canny_detector, 200.0);
  n.param("center_detection_threshold", center_detection_threshold, 100.0);
  n.param("min_radius", min_radius, 0);
  n.param("max_radius", max_radius, 0);

  ros::Subscriber scan_sub = n.subscribe("/map", 50, mapSubCallback);

  image_pub = n.advertise<sensor_msgs::Image>("image", 50);
  circles_pub = n.advertise<sensor_msgs::Image>("circles_image", 50);

  cv_img.header.frame_id = "map_image";
  cv_img.encoding = sensor_msgs::image_encodings::MONO8;

  cv_circlesImg.header.frame_id = "map_circles";
  cv_circlesImg.encoding = sensor_msgs::image_encodings::MONO8;

  while (n.ok()) {
    ros::spinOnce();
  }
}

void mapSubCallback(const nav_msgs::OccupancyGridConstPtr& map) {
  int size_x = map->info.width;
  int size_y = map->info.height;

  if ((size_x < 3) || (size_y < 3) ) {
    ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
    return;
  }

  cv::Mat *map_mat  = &cv_img.image;

  // resize cv image if it doesn't have the same dimensions as the map
  if ( (map_mat->rows != size_y) && (map_mat->cols != size_x)) {
    *map_mat = cv::Mat(size_y, size_x, CV_8U);
  }

  cv_circlesImg.image = cv::Mat(size_y, size_x, CV_8U, 255);


  const std::vector<int8_t>& map_data(map->data);

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

  // Reduce the noise so we avoid false circle detection
  //cv::GaussianBlur(cv_circlesImg.image, cv_img.image, cv::Size(9, 9), 2, 2);

  std::vector<cv::Vec3f> circles;

  // Apply the Hough Transform to find the circles
  HoughCircles(cv_img.image,
               circles,
               CV_HOUGH_GRADIENT,
               inverse_ratio_of_resolution,
               minimum_distance_between_detected_centers,
               upper_threshold_for_canny_detector,
               center_detection_threshold,
               min_radius,
               max_radius);

  std::cout << circles.size() << " circles\n\n" << std::endl;

  // Draw the circles detected
  for (size_t i = 0; i < circles.size(); i++) {
    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);
    ROS_INFO_STREAM("Radius:" << radius);
    // circle center
    //cv::circle(cv_circlesImg.image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
    // circle outline
    cv::circle(cv_circlesImg.image, center, radius, 127, 30, 8, 0);
  }

  Canny(cv_img.image, cv_img.image, 500, 200);


  image_pub.publish(cv_img.toImageMsg());
  circles_pub.publish(cv_circlesImg.toImageMsg());

  return;
}
