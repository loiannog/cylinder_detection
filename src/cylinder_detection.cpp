#include "std_msgs/String.h"
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <imgproc.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
// Set dot characteristics for the auto detection

using namespace std;
using namespace ros;

void cylinder_detection::onInit(void) {
  ros::NodeHandle priv_nh(getPrivateNodeHandle());
  std::string path_file;
  priv_nh.param<string>("path_file", path_file,
                        "/home/xua/git/cylinder_detection/src");
  priv_nh.param<int>("image_threshold", image_threshold,
                     70);  // Surface of a dot to search in an area.
  priv_nh.param<double>("opt_sizePrecision", opt_sizePrecision, 0.65);
  priv_nh.param<double>("opt_grayLevelPrecision", opt_grayLevelPrecision, 0.85);
  priv_nh.param<double>("opt_ellipsoidShapePrecision",
                        opt_ellipsoidShapePrecision, 0.5);
  priv_nh.param<int>("height_min", height_min,
                     0);  // Coordinate (row) of the upper-left area corner.
  priv_nh.param<int>("height_max", height_max,
                     480);  // Height or the area in which a dot is searched.
  priv_nh.param<int>("width_min", width_min,
                     0);  // Coordinate (column) of the upper-left area corner.
  priv_nh.param<int>("width_max", width_max,
                     752);  // Width or the area in which a dot is searched.
  priv_nh.param<int>("height_min", height_min,
                     0);  // Coordinate (row) of the upper-left area corner.
  priv_nh.param<int>("GrayLevelMin", GrayLevelMin, 250);  // GrayLevel min.
  priv_nh.param<int>("GrayLevelMax", GrayLevelMax, 255);  // GrayLevel max.
  priv_nh.param<int>("Surface", Surface,
                     124);  // Surface of a dot to search in an area.
  priv_nh.param<double>("fx", fx,
                        621.755015);  // Surface of a dot to search in an area.
  priv_nh.param<double>("fy", fy,
                        617.402184);  // Surface of a dot to search in an area.
  priv_nh.param<double>("cx", cx,
                        395.913754);  // Surface of a dot to search in an area.
  priv_nh.param<double>("cy", cy, 60);  // Surface of a dot to search in an
                                        // area.
  priv_nh.param<double>("d0", d0,
                        -0.406827);  // Surface of a dot to search in an area.
  priv_nh.param<double>("d1", d1,
                        0.173936);  // Surface of a dot to search in an area.
  priv_nh.param<double>("d2", d2,
                        -6.1e-05);  // Surface of a dot to search in an area.
  priv_nh.param<double>("d3", d3,
                        -0.002139);  // Surface of a dot to search in an area.
  priv_nh.param<int>("thresh_threshold", thresh_threshold, 50);
  priv_nh.param<int>("maxThreshold", maxThreshold, 255);
  priv_nh.param<int>("CannyMeanMultiplier", CannyMeanMultiplier, 100);
  priv_nh.param<int>("CannyStddevMultiplier", CannyStddevMultiplier, 70);
  priv_nh.param<int>("Canny_kernel_size", Canny_kernel_size, 7);
  priv_nh.param<int>("rhoRes", rhoRes, 1);
  priv_nh.param<double>("thetaRes", thetaRes, 0.017453293);
  priv_nh.param<int>("HoughThresh", HoughThresh, 10);
  priv_nh.param<int>("minLineLength", minLineLength, 25);
  priv_nh.param<int>("maxLineGap", maxLineGap, 25);
  priv_nh.param<int>("kernelSize", kernelSize, 3);
  priv_nh.param<int>("sigmaX", sigmaX, 0);
  priv_nh.param<int>("nbLines", nbLines, 2);
  priv_nh.param<int>("method", method, 1);

  /*cvNamedWindow("Original image");
    cvNamedWindow("Threshold");
    cvNamedWindow("Reduced");
    cvNamedWindow("Blurred");
    cvStartWindowThread();*/
  image_transport::ImageTransport it(priv_nh);
  image_thresholded_pub_ = it.advertise("/image_detected", 1);
  image_thresholded_original_pub_ = it.advertise("/image_detected_theshold", 1);
  image_lines_pub_ = it.advertise("/image_with_lines", 1);

  // cylinder_pos_pub_ =
  // priv_nh.advertise<std_msgs::Float32MultiArray>("/cylinder_position_testing",
  // 5);
  cylinder_pos_pub_ =
      priv_nh.advertise<cylinder_msgs::ImageFeatures>("cylinder_features", 5);
  image_transport::TransportHints hints(
      "raw", ros::TransportHints().tcpNoDelay(), priv_nh);
  sub_camera_ =
      it.subscribe("image", 1, &cylinder_detection::camera_callback, this);
}

void cylinder_detection::camera_callback(
    const sensor_msgs::ImageConstPtr& img) {
  static bool initialized = false;
  static ros::Time initial_timestamp;
  try {

    if (!initialized) {
      initial_timestamp = img->header.stamp;
      // memcpy(&P[0],&(c->P[0]),8*sizeof(double));
      initialized = true;
    }
    // cv::Mat src(cv::Size(img->width, img->height), CV_8UC1,
    // const_cast<uchar*>(&img->data[0]), img->step);
    // cv::Mat src_sub = src.rowRange(src.rows/2 - 60, src.rows/2 + 60);
    cv::Mat src = cv_bridge::toCvShare(img)->image;

    imgproc_visp(src, img->header.stamp);
  }
  catch (const std::bad_alloc& e) {
    cout << "error:" << e.what() << endl;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cylinder_detection, nodelet::Nodelet);
