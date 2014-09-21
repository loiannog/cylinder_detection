#ifndef _IMGPROC_H_
#define _IMGPROC_H_

#include <iostream>
#include "opencv2/core/core.hpp"
using namespace std;
#include <math.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace cv;
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visp/vpConfig.h>
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpMeLine.h>
#include <visp/vpOpenCVGrabber.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <cylinder_msgs/ImageFeatures.h>
#include <iostream>
#define pi 3.141592653589


class cylinder_detection : public nodelet::Nodelet
{
 public:
	  int image_threshold;
	  double opt_sizePrecision;
	  double opt_grayLevelPrecision;
	  double opt_ellipsoidShapePrecision;
	  int height_min;
	  int height_max;
	  int width_min;
	  int width_max;
	  int GrayLevelMin;
	  int GrayLevelMax;
	  int Surface;
	  double fx;
	  double fy;
	  double cx;
	  double cy;
	  double d0;
	  double d1;
	  double d2;
	  double d3;
	  int lowThreshold;
	  int thresh_threshold;
	  int maxThreshold;
	  int maxCannyThreshold;
	  int aperture_size;
	  double thetaRes;
	  int HoughThresh;
	  int minLineLength;
	  int maxLineGap;
	  int kernelSize;
	  int rhoRes;
	  int sigmaX;
	   

	  image_transport::Publisher image_thresholded_pub_;
	    ros::Publisher cylinder_pos_pub_;
	    cylinder_msgs::ImageFeatures detected_features;

	void onInit(void);
	int nbLines;
        vpDot2 dot_search;
        vpImagePoint init_point_blob;
	std::vector<vpMeLine> line_buffer;
	void imgproc_visp(const cv::Mat &img);
        void imgproc_opencv(const cv::Mat &img);


 private:
  void camera_callback(const sensor_msgs::Image::ConstPtr &img);
  image_transport::CameraSubscriber sub_camera_;
  ros::Subscriber sub_image_;
};

typedef struct _Patch
{
  double x1;
  double y1;
  double x2;
  double y2;
  double orientation;
  ros::Time time_computation;
}Patch;


#endif
