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
#define automatic_detection
// Set dot characteristics for the auto detection
using namespace std;
using namespace cv;
using namespace ros;
bool points_init = false;
   vpDisplayOpenCV d;

//#define GUI
//Given a line (or 4 coordinates, x1, y1, x2, y2), the formula returns the line's length, or the distance between the points.
float distanceFormula(Vec4i l)
{
	float distance = 0.0;
	distance = sqrt(pow(abs(l[0]-l[2]),2)+pow(abs(l[1]-l[3]),2));
	return distance;
}

void cylinder_detection::imgproc_visp(const Mat &src)
{
       	double begin = ros::Time::now().toSec();
  Mat blurred, thresholded, dst, cdst; //Image matrices
  GaussianBlur(src, blurred, Size(kernelSize,kernelSize), sigmaX);//clean the image
  threshold(blurred, thresholded, thresh_threshold, maxThreshold, THRESH_TOZERO);//threshold the image
  

   vpImage<unsigned char> I;
   vpImageConvert::convert(thresholded, I);
   d.init(I, 0, 0, "") ;
   
   vpDisplay::display(I);


   vpMe me;
   vpMeLine line[nbLines];

  //Set the tracking parameters.
   me.setRange(30);//set the search range on both sides of the reference pixel
   me.setSampleStep(5);//set the minimum distance in pixel between two discretized points.
   //each pixel along the normal we will compute the oriented convolution
   me.setThreshold(15000);//the pixel that will be selected by the moving edges algorithm will be the one that has a convolution higher than 15000
  //Initialize the tracking.
   	  std::list<vpDot2> list_d;//list of elements in constrast respect ot the background

	  
//initialization
if(!points_init)
	  {	  
#ifdef automatic_detection
	   /*dot_search.setGrayLevelMin(GrayLevelMin);
	  dot_search.setGrayLevelMax(GrayLevelMax);
	  dot_search.setGrayLevelPrecision(opt_grayLevelPrecision);
	  dot_search.setSizePrecision(opt_sizePrecision);
	  dot_search.setEllipsoidShapePrecision(opt_ellipsoidShapePrecision);*/
	  try
	   {
		  while(list_d.size() == 0){
		  dot_search.searchDotsInArea(I, width_min,  height_min,  width_max, height_max, list_d) ;
		  cout<<"searching the first dot"<<endl;
		  dot_search = list_d.back();
		  init_point_blob.set_i(dot_search.getBBox().getCenter().get_i());
		  init_point_blob.set_j(dot_search.getBBox().getCenter().get_j());
		  cout<<"blob init:"<<dot_search.getBBox().getCenter().get_i()<<" "<<dot_search.getBBox().getCenter().get_j()<<endl;
		  	}
	   }
	   
	  catch (int e)
	   {
	     cout << "An exception occurred. Exception Nr. " << e << endl;
	   }  
	 
#else
         dot_search.initTracking(I);
#endif
	  
	  
	  

	    
	    vector<vpImagePoint> init_points;
	    init_points.resize(4);
	    vpRect box = dot_search.getBBox();
	    init_points[0].set_ij(box.getBottomRight().get_i(), box.getBottomRight().get_j());
	    init_points[1].set_ij(box.getBottomRight().get_i() - box.getHeight() + 4, dot_search.getCog().get_j());
	    init_points[2].set_ij(box.getTopLeft().get_i()+3, box.getTopLeft().get_j());
	    init_points[3].set_ij(box.getTopLeft().get_i() + box.getHeight() - 4, box.getTopLeft().get_j());
	    
	    cout<<"valuesr:"<<box.getBottomRight().get_i()<<" "<<box.getBottomRight().get_j()<<endl;
	    cout<<"valuesl:"<<box.getTopLeft().get_i()<<" "<<box.getTopLeft().get_j()<<endl;
	    
       int k = 0;
	for (int i =0; i < 2; i++)
	{
	  line[i].setMe(&me);
	  line[i].setDisplay(vpMeSite::RANGE_RESULT);
	  line[i].initTracking(I,init_points[k],init_points[k+1]);
	  line[i].track(I);
	  line_buffer.push_back(line[i]);
	  points_init = true;
	  k = k+2;
	  cout<<i<<endl;
	}
	      	 //dot_search.track(I);//track the dot
#ifdef automatic_detection
           dot_search.initTracking(I,init_point_blob);
#endif

	    dot_search.track(I);//track the dot
      }

	    else { //Track the line.

	     try
	       {
		// track the blob
		dot_search.track(I);
		dot_search.display(I, vpColor::red) ;
		}
		  catch (const std::exception &e)
		{
	       		  cout<<"tracking failed"<<endl;
		  }
	      for (int i =0; i < nbLines; i++)
	      {
	     try
	       {
		line_buffer[i].track(I);
		line_buffer[i].display(I, vpColor::green) ;
		}
		  catch (const std::exception &e)
		{
	       		  cout<<"tracking failed"<<endl;
		  }
	      }
	      }
              double finalTime = ros::Time::now().toSec();
	      cout<<1.0/(finalTime-begin)<<endl;
	      vpDisplay::flush(I);


}

//Function where all visual cylinder detection takes place
void cylinder_detection::imgproc_opencv(const Mat &src)
{
	//Declarations
        double begin = ros::Time::now().toSec();
	Mat blurred, thresholded, dst, cdst; //Image matrices
	vector<Vec4i> lines; //Vector to hold all lines return by a Hough Transform
	vector<Vec4i> buffer1; //Vector to hold the first line (2-D vector to make it simple to sort)
	vector<Vec4i> buffer2; //Vector to hold the second line
	Vec4i maxL; //Holds coordinates of the largest line
	Vec4i otherLine; //Holds coordinates of the line parallel to the largest.

	//Image alterations
	GaussianBlur(src, blurred, Size(kernelSize,kernelSize), sigmaX);//clean the image
	threshold(blurred, thresholded, thresh_threshold, maxThreshold, THRESH_TOZERO);//threshold the image
	Canny(thresholded, dst, lowThreshold, maxCannyThreshold, aperture_size); //Perform the Canny edge detection on the image.
  	HoughLinesP(dst, lines, rhoRes, thetaRes, HoughThresh, minLineLength, maxLineGap );//Perform hough line transform on the imgae
	//cout<<"processing time:"<<(ros::Time::now().toSec()-begin)<<endl;
	
	//Looks for the largest line that was returned by Hough Line Transform
  	for( size_t i = 0; i < lines.size(); i++ )
  	{
    		Vec4i l = lines[i];
    		if(buffer1.empty())
    		{
			buffer1.push_back(l);
    		}
    		else
    		{
	    		if(distanceFormula(l)>distanceFormula(buffer1[0]))
	    		{
			   buffer1.pop_back();
			   buffer1.push_back(l);
	    		}
    		}
  	} 
	//If there was a line detected at all, this now looks for the line parallel to that line
  	if(buffer1.size()!=0)
  	{
	  	maxL = buffer1[0]; //Largest line in image
	  	float maxLAngle = atan2((maxL[3]-maxL[1]),(maxL[2]-maxL[0]))* 180 / CV_PI;
	  	vector<float> maxLMid; //Holdes midpoint of the largest line

	  	maxLMid.push_back(((maxL[2]+maxL[0])/2.0));//Calculates and stores midpoint
	  	maxLMid.push_back(((maxL[3]+maxL[1])/2.0));
	  	for(size_t i = 0;i<lines.size();i++) //Loops through all the lines to find the one parallel
	  	{
			Vec4i l = lines[i];
			float lineAngle = atan2((l[3]-l[1]),(l[2]-l[0]))* 180 / CV_PI; //Calculate angle of current line

			vector<float> lMid;
	  		lMid.push_back(((l[2]+l[0])/2.0)); //Store and calculate the midpoint of the new line
	  		lMid.push_back(((l[3]+l[1])/2.0));
			Vec4i lineDistance; //Create an imaginary line between the midpoint of the largest line and the midpoint of the current line
			//We will calculate this distance and then use that to filter out all parallel lines that are parallel to the largest line but are also more or less
			//on top of it, so that we only get the cylinder's parallel line. (Before it would look like we were returning one line, but it was actually two lines
			//so close to each other they looked identical.
			lineDistance[0] = lMid[0];
			lineDistance[1] = lMid[1];
			lineDistance[2] = maxLMid[0];
			lineDistance[3] = maxLMid[1];
			
			//Logic: Check if the difference in the angles of the two lines are within 20 degrees to see if they are almost parallel. Then, check how far away
			//these lines are to make sure that they aren't superimposed. Then check to see if the x distance is some distance away to make sure these two lines
			//are not superimposed, and repeat for the y distance. Finally, make sure the two lines are not exactly the same.
			//if(abs(lineAngle-maxLAngle)<20.0 && distanceFormula(lineDistance)>20.0 && distanceFormula(lineDistance)<100.0 && (abs(lineDistance[2]-lineDistance[0])>10.0) && (abs(lineDistance[3]-lineDistance[1])>10.0) && abs(lineAngle-maxLAngle)!=0.0)
			if(abs(lineAngle-maxLAngle) < 20.0 && distanceFormula(lineDistance)*cos(lineAngle) > 10.0 && abs(lineAngle-maxLAngle) != 0.0)

			{	
				if(buffer2.size()>0)
				{
					Vec4i largestBufferLine = buffer2.back();
					float largestLength = distanceFormula(largestBufferLine);
					if(largestLength<distanceFormula(l))
					{
					  buffer2.pop_back();
					  buffer2.push_back(l);
					}
					 //Remove the previous line (if there are multiple parallel lines, it doesn't matter which we use.)
				}
				else
				{
				  buffer2.push_back(l); //Add the parallel line
				}
			}
	  	}
	  	if(buffer2.size()>0) //Make sure the program actually detected a parallel line
	  	{
			otherLine = buffer2.back(); //Store the parallel line
	  	}
  	}
  	std_msgs::Float32MultiArray array; //Array to store the data
  	if(buffer2.size()>0) //Make sure we have generated both a line and its parallel.
  	{
         	array.data.clear();
        	array.data.push_back(maxL[0]);
		array.data.push_back(maxL[1]);
		array.data.push_back(maxL[2]);
		array.data.push_back(maxL[3]);
		array.data.push_back(otherLine[0]);
		array.data.push_back(otherLine[1]);
		array.data.push_back(otherLine[2]);
		array.data.push_back(otherLine[3]); //Store data
		cout<<"data:"<<array.data[0]<<endl;
		cylinder_pos_pub_.publish(array); //Publish data
  	}
	double finalTime = ros::Time::now().toSec();
	
		cvtColor(dst, cdst, CV_GRAY2BGR);

// 	    for( size_t i = 0; i < lines.size(); i++ )
//     {
//         line( cdst, Point(lines[i][0], lines[i][1]),
//             Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
//     }
       line( cdst, Point(maxL[0], maxL[1]), Point(maxL[2], maxL[3]), Scalar(0,0,255), 2, CV_AA);
       line( cdst, Point(otherLine[0], otherLine[1]), Point(otherLine[2], otherLine[3]), Scalar(0,0,255), 2, CV_AA); //Draw the lines
	
	//cout<<1.0/(finalTime-begin)<<endl;
  	cv::imshow("thresholded",thresholded); //Show the resulting image
	cv::imshow("image",cdst); //Show the resulting image
  	cv::waitKey(0);
}
