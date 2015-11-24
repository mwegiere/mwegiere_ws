#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void callback(const sensor_msgs::ImageConstPtr& img) {
  cv::Mat image = cv_bridge::toCvShare(img, "bgr8")->image.clone();
  std::cout << image.size() << std::endl;
}

int main(int argc, char **argv) {

  static char * tmp = NULL;
  static int tmpi;
  ros::init(tmpi, &tmp, "image_listener", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/camera/image_color", 1000, callback);

  ros::spin();

  return 0;
}
