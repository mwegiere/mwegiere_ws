#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <serwo/SerwoInfo.h>

class ImageProcessing {
 public:
  ImageProcessing(ros::NodeHandle &nh);
  //-----------------------------------
  //++++input of SolvePnP++++
  //generating camera image points
  std::vector<cv::Point2f> Generate2DPoints();
  //generating real object points
  std::vector<cv::Point3f> Generate3DPoints();
  //camera image points
  std::vector<cv::Point2f> imagePoints;
  //real object points
  std::vector<cv::Point3f> objectPoints;
  //distort coefficients
  cv::Mat distCoeffs;
  //camera matrix
  cv::Mat cameraMatrix;
  //-----------------------------------
  //++++output of SolvePnP++++
  //rotation vector
  cv::Mat_<double> rvec;
  //translation vector
  cv::Mat_<double> tvec;
  //-----------------------------------

  //1/0 depends on diod grid grid was found / not found
  int found;

  //-----------------------------------
  //mesage to be published (grid position in camera frame)
  serwo::SerwoInfo msg;
  //publisher (grid position in camera frame)
  ros::Publisher grid_info_pub;

 private:
  //original image from camera
  cv::Mat sourceImage;
  ros::NodeHandle nh_;
  //subscribe an image from camera topic
  ros::Subscriber imageSubscriber;
  //callback function used for subscribe image from topic
  void callback(const sensor_msgs::ImageConstPtr& img);

};
void ImageProcessing::callback(const sensor_msgs::ImageConstPtr& img) {
  sourceImage = cv_bridge::toCvShare(img, "bgr8")->image.clone();
}

std::vector<cv::Point2f> ImageProcessing::Generate2DPoints() {
  cv::Mat image = sourceImage;

  double sumaB = 0;
  double sumaG = 0;
  double sumaR = 0;

  //wsp maksymalnych składowych
  cv::Point2f wsp_maxB;
  cv::Point2f wsp_maxG;
  cv::Point2f wsp_maxR;
  cv::Point2f wsp_maxW;

  //maksymalne składowe
  int maxB = 0;
  double maxBminG = 255;
  double maxBminR = 255;

  int maxG = 0;
  double maxGminB = 255;
  double maxGminR = 255;

  int maxR = 0;
  double maxRminB = 255;
  double maxRminG = 255;

  int maxW = 0;

  int rows = image.size().height;
  int cols = image.size().width;

  uchar * ptr;
  for (int i = 3; i < rows; ++i) {
    ptr = image.ptr(i);
    for (int j = 3; j < cols; ++j) {
      int b = ptr[3 * j];
      int g = ptr[3 * j + 1];
      int r = ptr[3 * j + 2];

      if (b - (r + g) > maxB) {
        wsp_maxB.y = i;
        wsp_maxB.x = j;
        maxB = b - (r + g);
      }

      if (g - (r + b) > maxG) {
        wsp_maxG.y = i;
        wsp_maxG.x = j;
        maxG = g - (r + b);
      }

      if (r - (g + b) > maxR) {
        wsp_maxR.y = i;
        wsp_maxR.x = j;
        maxR = r - (g + b);
      }

      if (r + g + b > maxW) {
        wsp_maxW.y = i;
        wsp_maxW.x = j;
        maxW = r + g + b;
      }
    }
  }

  int prog = 50;
  std::vector<cv::Point2f> gridPoints;
  cv::Point2f wsp_zero;
  wsp_zero.x = 0.0;
  wsp_zero.y = 0.0;

  if (maxB > prog && maxG > prog && maxR > prog && maxW > prog) {
    found = 1;
    /*gridPoints.push_back(wsp_maxB);
     gridPoints.push_back(wsp_maxG);
     gridPoints.push_back(wsp_maxR);
     gridPoints.push_back(wsp_maxW);*/
  } else {
    found = 0;
  }
  gridPoints.push_back(wsp_maxB);
  gridPoints.push_back(wsp_maxG);
  gridPoints.push_back(wsp_maxR);
  gridPoints.push_back(wsp_maxW);
  return gridPoints;
}

ImageProcessing::ImageProcessing(ros::NodeHandle &nh) {
  nh_ = nh;
  imageSubscriber = nh_.subscribe("/camera/image_color", 1000,
                                  &ImageProcessing::callback, this);

  grid_info_pub = nh_.advertise<serwo::SerwoInfo>("object_seen_by_camera", 1);

  //3D model points are constant
  objectPoints = Generate3DPoints();

  distCoeffs = cv::Mat(5, 1, cv::DataType<double>::type);
  distCoeffs.at<double>(0) = -0.432862;
  distCoeffs.at<double>(1) = 0.206828;
  distCoeffs.at<double>(2) = -0.005944;
  distCoeffs.at<double>(3) = 0.003594;
  distCoeffs.at<double>(4) = 0.0;

  cameraMatrix = cv::Mat(3, 3, cv::DataType<double>::type);
  cameraMatrix.at<double>(0, 0) = 1097.309750;
  cameraMatrix.at<double>(0, 1) = 0.0;
  cameraMatrix.at<double>(0, 2) = 631.666708;
  cameraMatrix.at<double>(1, 0) = 0.0;
  cameraMatrix.at<double>(1, 1) = 1091.041004;
  cameraMatrix.at<double>(1, 2) = 518.507823;
  cameraMatrix.at<double>(2, 0) = 0.0;
  cameraMatrix.at<double>(2, 1) = 0.0;
  cameraMatrix.at<double>(2, 2) = 1.0;

  rvec = cv::Mat(3, 1, cv::DataType<double>::type);
  tvec = cv::Mat(3, 1, cv::DataType<double>::type);
}

std::vector<cv::Point3f> ImageProcessing::Generate3DPoints() {
  std::vector<cv::Point3f> points;
  points.push_back(cv::Point3f(0.04813, 0.00802, 0));  //B
  points.push_back(cv::Point3f(0.00943, 0.0457, 0));  //G
  points.push_back(cv::Point3f(0.090827, 0.04535, 0));  //R
  points.push_back(cv::Point3f(0.09066, 0.004626, 0));  //W
  return points;
}

int main(int argc, char **argv) {

  static char * tmp = NULL;
  static int tmpi;
  ros::init(tmpi, &tmp, "image_processing", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ImageProcessing imageProcessing(nh);

  while (ros::ok()) {
    //updating image points
    imageProcessing.imagePoints = imageProcessing.Generate2DPoints();
    //solvePnP
    cv::solvePnP(imageProcessing.objectPoints, imageProcessing.imagePoints,
                 imageProcessing.cameraMatrix, imageProcessing.distCoeffs,
                 imageProcessing.rvec, imageProcessing.tvec);
    //change rvec to matrix
    cv::Mat_<double> rotationMatrix;
    cv::Rodrigues(imageProcessing.rvec, rotationMatrix);
    //final matrix (grid position in camera frame)
    cv::Mat pattern_pose =
        (cv::Mat_<double>(4, 4) << rotationMatrix(0, 0), rotationMatrix(0, 1), rotationMatrix(
            0, 2), imageProcessing.tvec(0), rotationMatrix(1, 0), rotationMatrix(
            1, 1), rotationMatrix(1, 2), imageProcessing.tvec(1), rotationMatrix(
            2, 0), rotationMatrix(2, 1), rotationMatrix(2, 2), imageProcessing
            .tvec(2), 0, 0, 0, 1);

    /*printf("[ %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f]\n ", rotationMatrix(0,0), rotationMatrix(0,1), rotationMatrix(0,2), imageProcessing.tvec(0),
     rotationMatrix(1,0), rotationMatrix(1,1), rotationMatrix(1,2), imageProcessing.tvec(1),
     rotationMatrix(2,0), rotationMatrix(2,1), rotationMatrix(2,2), imageProcessing.tvec(2),
     0.0, 0.0, 0.0, 1.0);*/
    //push pattern_pose to vector
    std::vector<double> pattern_pose_vector;
    pattern_pose_vector.push_back(rotationMatrix(0, 0));
    pattern_pose_vector.push_back(rotationMatrix(0, 1));
    pattern_pose_vector.push_back(rotationMatrix(0, 2));
    pattern_pose_vector.push_back(imageProcessing.tvec(0));
    pattern_pose_vector.push_back(rotationMatrix(1, 0));
    pattern_pose_vector.push_back(rotationMatrix(1, 1));
    pattern_pose_vector.push_back(rotationMatrix(1, 2));
    pattern_pose_vector.push_back(imageProcessing.tvec(1));
    pattern_pose_vector.push_back(rotationMatrix(2, 0));
    pattern_pose_vector.push_back(rotationMatrix(2, 1));
    pattern_pose_vector.push_back(rotationMatrix(2, 2));
    pattern_pose_vector.push_back(imageProcessing.tvec(2));
    pattern_pose_vector.push_back(0.0);
    pattern_pose_vector.push_back(0.0);
    pattern_pose_vector.push_back(0.0);
    pattern_pose_vector.push_back(1.0);
    //create message
    imageProcessing.msg.matrix = pattern_pose_vector;
    imageProcessing.msg.found = imageProcessing.found;
    imageProcessing.msg.out_time_nsec_pocz = 0;
    imageProcessing.msg.out_time_sec_pocz = 0;
    imageProcessing.msg.out_time_nsec_kon = 0;
    imageProcessing.msg.out_time_sec_kon = 0;
    //publish message
    imageProcessing.grid_info_pub.publish(imageProcessing.msg);

    ros::spinOnce();
  }
  return 0;
}
