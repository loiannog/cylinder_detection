#include "imgproc.h"
#include <iostream>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <boost/array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#define automatic_detection
#define hough_detection
// Set dot characteristics for the auto detection
using namespace std;
using namespace cv;
using namespace ros;
bool visualization = false;
bool debug_vis = false;
bool points_init = false;
vpDisplayOpenCV d;
int counter;
//#define GUI
// Given a line (or 4 coordinates, x1, y1, x2, y2), the formula returns the
// line's length, or the distance between the points.
float distanceFormula(Vec4i l) {
  float distance = 0.0;
  distance = sqrt(pow(abs(l[0] - l[2]), 2) + pow(abs(l[1] - l[3]), 2));
  return distance;
}

float distanceFormularobust(Vec4i l, double diff_rho) {
  float distance_mid_points = 0.0;
  distance_mid_points =
      sqrt(pow(abs(l[0] - l[2]), 2) + pow(abs(l[1] - l[3]), 2));
  double distance = fmin(diff_rho, distance_mid_points);
  return distance;
}

void cylinder_detection::imgproc_visp(const Mat &src,
                                      const ros::Time &frame_time) {

  double begin = ros::Time::now().toSec();
  Mat blurred, thresholded, dst, cdst;  // Image matrices
  GaussianBlur(src, blurred, Size(kernelSize, kernelSize),
               sigmaX);  // clean the image
  threshold(blurred, thresholded, thresh_threshold, maxThreshold,
            THRESH_BINARY);  // threshold the image
  /// Canny detector
  //int const max_lowThreshold = 100;
//int ratio = 3;
//int kernel_size = 3;
  //Canny( blurred, thresholded, lowThreshold, lowThreshold*ratio, kernel_size );

  vpImage<unsigned char> I;
  vpImageConvert::convert(thresholded, I);

  if (visualization == true) {
    d.init(I, 0, 0, "");
    vpDisplay::display(I);
  }

  // vpMeLine line[nbLines];

  // Initialize the tracking.
  std::list<vpDot2>
      list_d;  // list of elements in constrast respect ot the background

  // initialize parameters for the dot
  dot_search.setGrayLevelMin(GrayLevelMin);
  dot_search.setGrayLevelMax(GrayLevelMax);
  // dot_search.setGrayLevelPrecision(opt_grayLevelPrecision);
  dot_search.setEllipsoidShapePrecision(opt_ellipsoidShapePrecision);
  // dot_search.setSizePrecision(opt_sizePrecision);
  vector<vpImagePoint> init_points;
  init_points.resize(4);

  // initialization
  if (!points_init) {
    // Set the tracking parameters.
    me.setRange(
        30);  // set the search range on both sides of the reference pixel
    // me.setSampleStep(4);//set the minimum distance in pixel between two
    // discretized points.
    // each pixel along the normal we will compute the oriented convolution
    me.setThreshold(
        15000);  // the pixel that will be selected by the moving edges
                 // algorithm will be the one that has a convolution
                 // higher than 15000
    me.setNbTotalSample(700);
    me.setPointsToTrack(700);

    if (method == 0) {
      try {
        dot_search.initTracking(I);
      }

      catch (int e) {
        cout << "An exception occurred. Exception Nr. " << e << endl;
      }

      vpRect box = dot_search.getBBox();
      // check the lenght to decide which is the long sides
      if (box.getWidth() > box.getHeight()) {
        init_points[0].set_ij(box.getBottomRight().get_i(),
                              box.getBottomRight().get_j());
        init_points[1].set_ij(
            box.getBottomRight().get_i(),
            box.getBottomRight().get_j() - box.getWidth() + 4);
        init_points[2].set_ij(box.getTopLeft().get_i(),
                              box.getTopLeft().get_j());
        init_points[3].set_ij(box.getTopLeft().get_i(),
                              box.getTopLeft().get_j() + box.getWidth() - 4);
      } else {
        init_points[0].set_ij(box.getBottomRight().get_i(),
                              box.getBottomRight().get_j());
        init_points[1].set_ij(
            box.getBottomRight().get_i() - box.getHeight() + 4,
            box.getBottomRight().get_j());
        init_points[2].set_ij(box.getTopLeft().get_i(),
                              box.getTopLeft().get_j());
        init_points[3].set_ij(box.getTopLeft().get_i() + box.getHeight() - 4,
                              box.getTopLeft().get_j());
      }
      cout << "valuesr:" << box.getBottomRight().get_i() << " "
           << box.getBottomRight().get_j() << endl;
      cout << "valuesl:" << box.getTopLeft().get_i() << " "
           << box.getTopLeft().get_j() << endl;

    } else if (method == 1) {
      // use Hough transform to initilize lines
      Vec4i P1;
      Vec4i P2;
      int size = 0;
      double max_area = 0;
      std::list<vpDot2>
          list_d;  // list of elements in constrast respect ot the background

      try {
        init_detection_hough(blurred, P1, P2, size);
        if (size != 2) return;

        init_points[0].set_ij(P1[1], P1[0]);
        init_points[1].set_ij(P1[3], P1[2]);
        init_points[2].set_ij(P2[1], P2[0]);
        init_points[3].set_ij(P2[3], P2[2]);
        cout << init_points[0] << endl;
        cout << init_points[1] << endl;
        cout << init_points[2] << endl;
        cout << init_points[3] << endl;
      }
      catch (int e) {
        cout << "An exception occurred detecting lines " << e << endl;
      }

    } else {  // method3

      vpRect box = dot_search.getBBox();
      // check the lenght to decide which is the long sides
      if (box.getWidth() > box.getHeight()) {
        init_points[0].set_ij(box.getBottomRight().get_i(),
                              box.getBottomRight().get_j());
        init_points[1].set_ij(
            box.getBottomRight().get_i(),
            box.getBottomRight().get_j() - box.getWidth() + 4);
        init_points[2].set_ij(box.getTopLeft().get_i(),
                              box.getTopLeft().get_j());
        init_points[3].set_ij(box.getTopLeft().get_i(),
                              box.getTopLeft().get_j() + box.getWidth() - 4);
      } else {
        init_points[0].set_ij(box.getBottomRight().get_i(),
                              box.getBottomRight().get_j());
        init_points[1].set_ij(
            box.getBottomRight().get_i() - box.getHeight() + 4,
            box.getBottomRight().get_j());
        init_points[2].set_ij(box.getTopLeft().get_i(),
                              box.getTopLeft().get_j());
        init_points[3].set_ij(box.getTopLeft().get_i() + box.getHeight() - 4,
                              box.getTopLeft().get_j());
      }
      cout << "valuesr:" << box.getBottomRight().get_i() << " "
           << box.getBottomRight().get_j() << endl;
      cout << "valuesl:" << box.getTopLeft().get_i() << " "
           << box.getTopLeft().get_j() << endl;
    }
    line_tracker_buff_thread.resize(nbLines);
    line_buffer.resize(nbLines);
    int k = 0;
    for (int i = 0; i < nbLines; i++) {
      line_buffer[i] = new vpMeLine();
      line_buffer[i]->setMe(&me);
      if (visualization == true)
        line_buffer[i]->setDisplay(vpMeSite::RANGE_RESULT);
      // line_buffer.push_back(line[i]);
      try {
        line_tracker_buff_thread[i] = new boost::thread(
            boost::bind(&vpMeLine::initTracking, line_buffer[i], I,
                        init_points[k], init_points[k + 1]));
      }
      catch (int e) {
        cout << "An exception occurred during initial line tracking " << e
             << endl;
      }
      points_init = true;
      k = k + 2;
    }

    for (int i = 0; i < nbLines; i++) {
      line_tracker_buff_thread[i]->join();
    }

    // dot_search.track(I);//track the dot
    // after initial tracking activate moment computation

  } else {

    for (int i = 0; i < nbLines; i++) {
      try {
	line_buffer[i]->seekExtremities(I);
        line_buffer[i]->setMe(&me);
        line_tracker_buff_thread[i] =
            new boost::thread(&vpMeLine::track, line_buffer[i], I);
         if(visualization == true)
         line_buffer[i]->display(I, vpColor::green) ;
      }
      catch (const std::exception &e) {
        cout << "tracking line failed" << endl;
      }
    }
  }
  for (int i = 0; i < nbLines; i++) {
    line_tracker_buff_thread[i]->join();
  }

  cv::Mat output_image(480, 752, CV_32F);
  cv_bridge::CvImage out_msg;

  // publish the image for the moment
  if (debug_vis == true) {
    vpImage<vpRGBa> Irgba;              // a color image
    vpImageConvert::convert(I, Irgba);  // convert a greyscale to a color image.
    vpImageConvert::convert(Irgba, output_image);
  }

  // project rho on the axes and transform to normalized image coordinates
  const cv::Mat cM =
      (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
  const cv::Mat Dl = (cv::Mat_<double>(4, 1) << 0, 0, 0, 0);
  vpImagePoint point11;
  vpImagePoint point12;
  vpImagePoint point21;
  vpImagePoint point22;
  line_buffer[0]->getExtremities(point11, point12);
  line_buffer[1]->getExtremities(point21, point22);
  vector<Point2f> P;
  P.resize(4);

  P[0].x = point11.get_j();
  P[0].y = point11.get_i();
  P[1].x = point12.get_j();
  P[1].y = point12.get_i();
  P[2].x = point21.get_j();
  P[2].y = point21.get_i();
  P[3].x = point22.get_j();
  P[3].y = point22.get_i();

  if (debug_vis == true) {
    circle(output_image, Point(cx, cy), 3, Scalar(255, 255, 0), -1);
    cv::line(output_image, Point(P[0].x, P[0].y), Point(P[1].x, P[1].y),
             Scalar(0, 0, 255), 2, CV_AA);
    cv::line(output_image, Point(P[2].x, P[2].y), Point(P[3].x, P[3].y),
             Scalar(255, 0, 0), 2, CV_AA);  // Draw the lines
    sensor_msgs::Image output_ros;
    // cv_bridge::CvImage out_msg;
    out_msg.encoding = sensor_msgs::image_encodings::RGB8;  // Or whatever
  }
  vector<Point2f> dst_lines_point;
  dst_lines_point.resize(4);
  undistortPoints(P, dst_lines_point, cM, Dl);
  // Computes the angle of the line
  double theta1 = atan2(dst_lines_point[1].y - dst_lines_point[0].y,
                        dst_lines_point[1].x - dst_lines_point[0].x);
  double theta2 = atan2(dst_lines_point[3].y - dst_lines_point[2].y,
                        dst_lines_point[3].x - dst_lines_point[2].x);
  // Angle of the perpendicular to the line
  theta1 = theta1 + M_PI / 2;
  theta2 = theta2 + M_PI / 2;
  double norm_rho1 =
      dst_lines_point[0].x * cos(theta1) + dst_lines_point[0].y * sin(theta1);
  double norm_rho2 =
      dst_lines_point[2].x * cos(theta2) + dst_lines_point[2].y * sin(theta2);

  // Change for Chaumette convention
  /*while(theta1 < 0)
  {
    norm_rho1 = -norm_rho1;
    theta1 = theta1 + M_PI;
  }
  while(theta2 > 0)
  {
    norm_rho2 = -norm_rho2;
    theta2 = theta2 - M_PI;
  }*/

  // give the output
  detected_features.stamp = frame_time;
  detected_features.rho1 = norm_rho1;
  detected_features.theta1 = theta1;
  detected_features.rho2 = norm_rho2;
  detected_features.theta2 = theta2;

  vector<Point2f> T_P;
  // vector<Point2f> C_G;
  T_P.resize(2);
  T_P[0].x = P[1].x;
  T_P[0].y = P[1].y;
  T_P[1].x = P[3].x;
  T_P[1].y = P[3].y;
  if (visualization == true) {
    cv::imshow("output_image", output_image);  // Show the resulting image
    cv::waitKey(1);
  }
  if (debug_vis == true) {
    circle(output_image, Point(T_P[0].x, T_P[0].y), 3, Scalar(0, 255, 0), -1);
        circle(output_image, Point(T_P[1].x, T_P[1].y), 3, Scalar(0, 255, 0), -1);
    out_msg.image = output_image;  // Your cv::Mat
    image_thresholded_pub_.publish(out_msg.toImageMsg());
  }

  // undistort point
  vector<Point2f> dst_P;
  dst_P.resize(2);
//first tangent point
  undistortPoints(T_P, dst_P, cM, Dl);
  detected_features.b1.x =
      dst_P[0].x / sqrt(pow(dst_P[0].x, 2) + pow(dst_P[0].y, 2) + 1);
  detected_features.b1.y =
      dst_P[0].y / sqrt(pow(dst_P[0].x, 2) + pow(dst_P[0].y, 2) + 1);
  detected_features.b1.z = 1 / sqrt(pow(dst_P[0].x, 2) + pow(dst_P[0].y, 2) + 1);
  //second tangent point
    detected_features.b2.x =
      dst_P[1].x / sqrt(pow(dst_P[1].x, 2) + pow(dst_P[1].y, 2) + 1);
  detected_features.b2.y =
      dst_P[1].y / sqrt(pow(dst_P[1].x, 2) + pow(dst_P[1].y, 2) + 1);
  detected_features.b2.z = 1 / sqrt(pow(dst_P[1].x, 2) + pow(dst_P[1].y, 2) + 1);
  cylinder_pos_pub_.publish(detected_features);
  if (visualization == true) vpDisplay::flush(I);
}

// initialial detection using hough transform
void cylinder_detection::init_detection_hough(const Mat &src, Vec4i &P1,
                                              Vec4i &P2, int &size) {
  // Declarations
  double begin = ros::Time::now().toSec();
  Mat dst, cdst;        // Image matrices
  vector<Vec4i> lines;  // Vector to hold all lines return by a Hough Transform
  vector<Vec4i>
      buffer1;  // Vector to hold the first line (2-D vector to make it
                // simple to sort)
  vector<Vec4i> buffer2;  // Vector to hold the second line
  Vec4i maxL;             // Holds coordinates of the largest line
  Vec4i otherLine;  // Holds coordinates of the line parallel to the largest.

  // Image alterations
  Canny(src, dst, lowThreshold, maxCannyThreshold,
        aperture_size);  // Perform the Canny edge detection on the image.
  HoughLinesP(dst, lines, rhoRes, thetaRes, HoughThresh, minLineLength,
              maxLineGap);  // Perform hough line transform on the imgae
  // cout<<"processing time:"<<(ros::Time::now().toSec()-begin)<<endl;

  // cout << "Number of lines: " << lines.size() << endl;
  // Looks for the largest line that was returned by Hough Line Transform
  for (size_t i = 0; i < lines.size(); i++) {
    Vec4i l = lines[i];
    if (buffer1.empty()) {
      buffer1.push_back(l);
      size = 1;
    } else {
      if (distanceFormula(l) > distanceFormula(buffer1[0])) {
        buffer1.pop_back();
        buffer1.push_back(l);
      }
    }
  }

  // If there was a line detected at all, this now looks for the line parallel
  // to that line
  if (buffer1.size() != 0) {
    maxL = buffer1[0];  // Largest line in image
    float maxLAngle =
        atan2((maxL[3] - maxL[1]), (maxL[2] - maxL[0])) * 180 / CV_PI;
    vector<float> maxLMid;  // Holdes midpoint of the largest line

    maxLMid.push_back(
        ((maxL[2] + maxL[0]) / 2.0));  // Calculates and stores midpoint
    maxLMid.push_back(((maxL[3] + maxL[1]) / 2.0));
    cout << "number other lines: " << lines.size() << endl;
    for (size_t i = 0; i < lines.size();
         i++)  // Loops through all the lines to find the one parallel
    {
      Vec4i l = lines[i];
      float lineAngle = atan2((l[3] - l[1]), (l[2] - l[0])) * 180 /
                        CV_PI;  // Calculate angle of current line

      vector<float> lMid;
      lMid.push_back(
          ((l[2] + l[0]) /
           2.0));  // Store and calculate the midpoint of the new line
      lMid.push_back(((l[3] + l[1]) / 2.0));
      Vec4i lineDistance;  // Create an imaginary line between the midpoint of
                           // the largest line and the midpoint of the current
                           // line
      // We will calculate this distance and then use that to filter out all
      // parallel lines that are parallel to the largest line but are also more
      // or less
      // on top of it, so that we only get the cylinder's parallel line. (Before
      // it would look like we were returning one line, but it was actually two
      // lines
      // so close to each other they looked identical.
      lineDistance[0] = lMid[0];
      lineDistance[1] = lMid[1];
      lineDistance[2] = maxLMid[0];
      lineDistance[3] = maxLMid[1];

      double theta_line1 = atan2(maxL[3] - maxL[1], maxL[2] - maxL[0]);
      double theta_line2 = atan2(l[3] - l[1], l[2] - l[0]);
      double rho_line1 =
          maxL[0] * cos(theta_line1) + maxL[1] * sin(theta_line1);
      double rho_line2 = l[0] * cos(theta_line2) + l[1] * sin(theta_line2);
      double diff_rho = fabs(rho_line1 - rho_line2);

      double length_maxL =
          sqrt(pow(maxL[3] - maxL[1], 2) + pow(maxL[2] - maxL[0], 2));
      double length_l = sqrt(pow(l[3] - l[1], 2) + pow(l[2] - l[0], 2));
      double diff_line_length = fabs(length_maxL - length_l);

      // Logic: Check if the difference in the angles of the two lines are
      // within 20 degrees to see if they are almost parallel. Then, check how
      // far away
      // these lines are to make sure that they aren't superimposed. Then check
      // to see if the x distance is some distance away to make sure these two
      // lines
      // are not superimposed, and repeat for the y distance. Finally, make sure
      // the two lines are not exactly the same.
      // if(abs(lineAngle-maxLAngle)<20.0 && distanceFormula(lineDistance)>20.0
      // && distanceFormula(lineDistance)<100.0 &&
      // (abs(lineDistance[2]-lineDistance[0])>10.0) &&
      // (abs(lineDistance[3]-lineDistance[1])>10.0) &&
      // abs(lineAngle-maxLAngle)!=0.0)
      // cout << "diff_line_length = " << diff_line_length << endl;

      if (abs(lineAngle - maxLAngle) < 20.0 && diff_line_length < 30 &&
          distanceFormularobust(lineDistance, diff_rho) > 10.0 &&
          abs(lineAngle - maxLAngle) != 0.0) {
        if (buffer2.size() > 0) {
          Vec4i largestBufferLine = buffer2.back();
          float largestLength = distanceFormula(largestBufferLine);
          if (largestLength <= distanceFormula(l)) {
            buffer2.pop_back();
            buffer2.push_back(l);
          }
          // Remove the previous line (if there are multiple parallel lines, it
          // doesn't matter which we use.)
        } else {
          buffer2.push_back(l);  // Add the parallel line
          size = 2;
        }
      }
    }
    if (buffer2.size() >
        0)  // Make sure the program actually detected a parallel line
    {
      otherLine = buffer2.back();  // Store the parallel line
    }
  }
  // cvtColor(dst, cdst, CV_GRAY2BGR);
  if (size == 2) {
    P1[0] = buffer1[0][0];
    P1[1] = buffer1[0][1];
    P1[2] = buffer1[0][2];
    P1[3] = buffer1[0][3];
    P2[0] = buffer2[0][0];
    P2[1] = buffer2[0][1];
    P2[2] = buffer2[0][2];
    P2[3] = buffer2[0][3];
    cout << P1[0] << " " << P1[1] << endl;
    cout << P1[2] << " " << P1[3] << endl;
    cout << P2[0] << " " << P2[1] << endl;
    cout << P2[2] << " " << P2[3] << endl;
  }

  // line( cdst, Point(maxL[0], maxL[1]), Point(maxL[2], maxL[3]),
  // Scalar(0,0,255), 2, CV_AA);
  // line( cdst, Point(otherLine[0], otherLine[1]), Point(otherLine[2],
  // otherLine[3]), Scalar(255,0,0), 2, CV_AA); //Draw the lines
  // circle(cdst, Point(P1[0],P1[1]), 5, Scalar(0,0,255));
  // circle(cdst, Point(P1[2],P1[3]), 5, Scalar(0,0,255));
  // circle(cdst, Point(P2[0],P2[1]), 5, Scalar(255,0,0));
  // circle(cdst, Point(P2[2],P2[3]), 5, Scalar(255,0,0));
  // cout<<1.0/(finalTime-begin)<<endl;
  // cv::imshow("image",cdst); //Show the resulting image
  // cv::waitKey(0);
}

// Function where all visual cylinder detection takes place
void cylinder_detection::imgproc_opencv(const Mat &src) {
  // Declarations
  double begin = ros::Time::now().toSec();
  Mat blurred, thresholded, dst, cdst;  // Image matrices
  vector<Vec4i> lines;  // Vector to hold all lines return by a Hough Transform
  vector<Vec4i>
      buffer1;  // Vector to hold the first line (2-D vector to make it
                // simple to sort)
  vector<Vec4i> buffer2;  // Vector to hold the second line
  Vec4i maxL;             // Holds coordinates of the largest line
  Vec4i otherLine;  // Holds coordinates of the line parallel to the largest.

  // Image alterations
  GaussianBlur(src, blurred, Size(kernelSize, kernelSize),
               sigmaX);  // clean the image
  threshold(blurred, thresholded, thresh_threshold, maxThreshold,
            THRESH_TOZERO);  // threshold the image
  Canny(thresholded, dst, lowThreshold, maxCannyThreshold,
        aperture_size);  // Perform the Canny edge detection on the image.
  HoughLinesP(dst, lines, rhoRes, thetaRes, HoughThresh, minLineLength,
              maxLineGap);  // Perform hough line transform on the imgae
  // cout<<"processing time:"<<(ros::Time::now().toSec()-begin)<<endl;

  // Looks for the largest line that was returned by Hough Line Transform
  for (size_t i = 0; i < lines.size(); i++) {
    Vec4i l = lines[i];
    if (buffer1.empty()) {
      buffer1.push_back(l);
    } else {
      if (distanceFormula(l) > distanceFormula(buffer1[0])) {
        buffer1.pop_back();
        buffer1.push_back(l);
      }
    }
  }
  // If there was a line detected at all, this now looks for the line parallel
  // to that line
  if (buffer1.size() != 0) {
    maxL = buffer1[0];  // Largest line in image
    float maxLAngle =
        atan2((maxL[3] - maxL[1]), (maxL[2] - maxL[0])) * 180 / CV_PI;
    vector<float> maxLMid;  // Holdes midpoint of the largest line

    maxLMid.push_back(
        ((maxL[2] + maxL[0]) / 2.0));  // Calculates and stores midpoint
    maxLMid.push_back(((maxL[3] + maxL[1]) / 2.0));
    for (size_t i = 0; i < lines.size();
         i++)  // Loops through all the lines to find the one parallel
    {
      Vec4i l = lines[i];
      float lineAngle = atan2((l[3] - l[1]), (l[2] - l[0])) * 180 /
                        CV_PI;  // Calculate angle of current line

      vector<float> lMid;
      lMid.push_back(
          ((l[2] + l[0]) /
           2.0));  // Store and calculate the midpoint of the new line
      lMid.push_back(((l[3] + l[1]) / 2.0));
      Vec4i lineDistance;  // Create an imaginary line between the midpoint of
                           // the largest line and the midpoint of the current
                           // line
      // We will calculate this distance and then use that to filter out all
      // parallel lines that are parallel to the largest line but are also more
      // or less
      // on top of it, so that we only get the cylinder's parallel line. (Before
      // it would look like we were returning one line, but it was actually two
      // lines
      // so close to each other they looked identical.
      lineDistance[0] = lMid[0];
      lineDistance[1] = lMid[1];
      lineDistance[2] = maxLMid[0];
      lineDistance[3] = maxLMid[1];

      // Logic: Check if the difference in the angles of the two lines are
      // within 20 degrees to see if they are almost parallel. Then, check how
      // far away
      // these lines are to make sure that they aren't superimposed. Then check
      // to see if the x distance is some distance away to make sure these two
      // lines
      // are not superimposed, and repeat for the y distance. Finally, make sure
      // the two lines are not exactly the same.
      // if(abs(lineAngle-maxLAngle)<20.0 && distanceFormula(lineDistance)>20.0
      // && distanceFormula(lineDistance)<100.0 &&
      // (abs(lineDistance[2]-lineDistance[0])>10.0) &&
      // (abs(lineDistance[3]-lineDistance[1])>10.0) &&
      // abs(lineAngle-maxLAngle)!=0.0)
      if (abs(lineAngle - maxLAngle) < 20.0 &&
          distanceFormula(lineDistance) * cos(lineAngle) > 10.0 &&
          abs(lineAngle - maxLAngle) != 0.0) {
        if (buffer2.size() > 0) {
          Vec4i largestBufferLine = buffer2.back();
          float largestLength = distanceFormula(largestBufferLine);
          if (largestLength < distanceFormula(l)) {
            buffer2.pop_back();
            buffer2.push_back(l);
          }
          // Remove the previous line (if there are multiple parallel lines, it
          // doesn't matter which we use.)
        } else {
          buffer2.push_back(l);  // Add the parallel line
        }
      }
    }
    if (buffer2.size() >
        0)  // Make sure the program actually detected a parallel line
    {
      otherLine = buffer2.back();  // Store the parallel line
    }
  }
  std_msgs::Float32MultiArray array;  // Array to store the data
  if (buffer2.size() >
      0)  // Make sure we have generated both a line and its parallel.
  {
    array.data.clear();
    array.data.push_back(maxL[0]);
    array.data.push_back(maxL[1]);
    array.data.push_back(maxL[2]);
    array.data.push_back(maxL[3]);
    array.data.push_back(otherLine[0]);
    array.data.push_back(otherLine[1]);
    array.data.push_back(otherLine[2]);
    array.data.push_back(otherLine[3]);  // Store data
    cout << "data:" << array.data[0] << endl;
    // cylinder_pos_pub_.publish(array); //Publish data
  }
  double finalTime = ros::Time::now().toSec();

  // cvtColor(dst, cdst, CV_GRAY2BGR);

  // 	    for( size_t i = 0; i < lines.size(); i++ )
  //     {
  //         line( cdst, Point(lines[i][0], lines[i][1]),
  //             Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
  //     }
  // line( cdst, Point(maxL[0], maxL[1]), Point(maxL[2], maxL[3]),
  // Scalar(0,0,255), 2, CV_AA);
  // line( cdst, Point(otherLine[0], otherLine[1]), Point(otherLine[2],
  // otherLine[3]), Scalar(0,0,255), 2, CV_AA); //Draw the lines

  // cout<<1.0/(finalTime-begin)<<endl;
  cv::imshow("thresholded", thresholded);  // Show the resulting image
  cv::imshow("image", cdst);               // Show the resulting image
  cv::waitKey(0);
}
