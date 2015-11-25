#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>

//#include "opencv2/core/core.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/highgui/highgui.hpp"

class ImageProcessing
{
public:
	ImageProcessing(ros::NodeHandle &nh);
private:
	//original image from camera
	cv::Mat sourceImage;
	ros::NodeHandle nh_;
	//subscribe an image from camera topic
	ros::Subscriber imageSubscriber;
	//callback function used for subscribe image from topic
	void callback(const sensor_msgs::ImageConstPtr& img);
	
	//-----------------------------------
	//input of SolvePnP
	//generating camera image points
	std::vector<cv::Point2f> Generate2DPoints();
	//generating real object points
	std::vector<cv::Point3f> Generate3DPoints();
	//camera image points
  	std::vector<cv::Point2f> imagePoints;
	//real object points
  	std::vector<cv::Point3f> objectPoints;
	cv::Mat distCoeffs;	
	cv::Mat cameraMatrix;
	//-----------------------------------
	//output of SolvePnP
	cv::Mat rvec;
	cv::Mat tvec;
	//-----------------------------------
}

ImageProcessing::ImageProcessing(ros::NodeHandle &nh)
{
	nh_ = nh;
	imageSubscriber = nh_.subscribe("/camera/image_color", 1000, callback);

  	imagePoints = Generate2DPoints();
  	objectPoints = Generate3DPoints();

  	distCoeffs = cv::Mat(4,1,cv::DataType<double>::type);
  	distCoeffs.at<double>(0) = 0;
  	distCoeffs.at<double>(1) = 0;
  	distCoeffs.at<double>(2) = 0;
  	distCoeffs.at<double>(3) = 0;  

  	cameraMatrix = cv::Mat(3,3,cv::DataType<double>::type);
  	cv::setIdentity(cameraMatrix);

  	rvec = cv::Mat(3,1,cv::DataType<double>::type);
  	tvec = cv::Mat(3,1,cv::DataType<double>::type);

}

void ImageProcessing::callback(const sensor_msgs::ImageConstPtr& img) 
{
 	sourceImage = cv_bridge::toCvShare(img, "bgr8")->image.clone();
  	std::cout << sourceImage.size() << std::endl;
}

std::vector<cv::Point2f> ImageProcessing::Generate2DPoints()
{
  	std::vector<cv::Point2f> points; 
	//usupelnic 
 	points.push_back(cv::Point2f(x,y));

  	return points;
}

std::vector<cv::Point3f> ImageProcessing::Generate3DPoints()
{
 	std::vector<cv::Point3f> points;
 	//usupelnic 
 	points.push_back(cv::Point3f(x,y,z));

  	return points;
}

int main(int argc, char **argv) {

	static char * tmp = NULL;
	static int tmpi;
	ros::init(tmpi, &tmp, "image_processing", ros::init_options::NoSigintHandler);  
	ImageProcessing imageProcessing(nh_);

_InputArray (const Mat &m)


	while (ros::ok()){
		cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
		cv::Mat pattern_pose = (cv::Mat_<double>(4, 4) <<
			rvec(0,0), rotationMatrix(0,1), rotationMatrix(0,2), tvec(0),
			rotationMatrix(1,0), rotationMatrix(1,1), rotationMatrix(1,2), tvec(1),
			rotationMatrix(2,0), rotationMatrix(2,1), rotationMatrix(2,2), tvec(2),
			0, 0, 0, 1);
		ros::spinOnce();
		//ros::spin();
	}
	return 0;
}
