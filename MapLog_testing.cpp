#include<iostream>
#include<stdlib.h>
#include<math.h>
#include "ros/ros.h"

//#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
//#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"

#include <opencv4/opencv2/core/core.hpp>     // opencv libraries
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

float MapRes=0.05;
cv::Mat MapLog = Mat::zeros(int(60/MapRes),int(60/MapRes),CV_8UC1);
int iter=0;

void LaneCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{

	int MapH,MapW;
	MapW=msg->info.width;   //number of columns (pixels)
	MapH=msg->info.height;  //number of rows (pixels)

	/*
	  0 1 2 3 4 5
	0 # # # # # #
	1 # # # # # #
	2 # # o 1 # # 		o is the origin of the map i.e. at 19,19
	3 # # 1 1 # #		1 denotes the map 
	4 # # # # # #
	5 # # # # # #


	*/
	int origin_x, origin_y;
	origin_x = int(19/MapRes);
	// origin_y = int(29/MapRes);
	origin_y = int(9/MapRes);
	
	Mat m = Mat::zeros(int(60/MapRes),int(60/MapRes),CV_8UC1);

	// if(first_iter)
	{
		for (int i=0; i<int(20/MapRes); i++)
	    {
	    	for (int j=0; j<MapW; j++)
		    {
		    	int i1=int(20/MapRes)-i-1;
		    	if(msg->data[i*MapW+j]!=0)
		    		m.at<uchar>(origin_y+i1,origin_x+j-int(MapW/4)) = 255; // in opencv mat object, the y axis starts from the top i.e the origin is on the top left corner
		    	// MapLog.at<uchar>(origin_y+i,origin_x+j-MapW/2) = ipm->data[i*MapW+j];  
		    }
	    }
	    // first_iter=false;
	}

	double theta=0; // in degrees
	if(iter>5 && iter<55)
		theta= 0.4; 

	double distance=5;

	///////////////////////////////////////////////////////////////////////
	// (origin_x+MapW/2, origin_y+int(20/MapRes)) is the origin of the car 
	///////////////////////////////////////////////////////////////////////

	// Mat m = Mat::zeros(500,500,CV_8UC1);
	// // for(int i=200;i<300;i++)
	// // 	for(int j=200;j<300;j++)
	// // 		m.at<uchar>(i,j)=255;
	// // imshow("img",m);
	// waitKey(0);

	// Mat mt = Mat::zeros(int(60/MapRes),int(60/MapRes),CV_8UC1);

// https://docs.opencv.org/master/d4/d61/tutorial_warp_affine.html

	// TRANSLATE

	// coordinates of the original triangle
	Point2f srcTri[3];							
    srcTri[0] = Point2f( 0.f, 0.f );					 // point A
    srcTri[1] = Point2f( m.cols - 1.f, 0.f ); 			 // point B
    srcTri[2] = Point2f( 0.f, m.rows - 1.f ); 			 // point C

    // coordinates of the same triangle in the transformed image
    Point2f dstTri[3];
    dstTri[0] = Point2f( 0.f, 0.f + distance);								// point A'
    dstTri[1] = Point2f( m.cols - 1.f, 0.f + distance );					// point B'
    dstTri[2] = Point2f( 0.f, m.rows - 1.f + distance );					// point C'
    Mat warp_mat = getAffineTransform( srcTri, dstTri );
    warpAffine( MapLog, MapLog, warp_mat, MapLog.size() );


    // ROTATE

    // Mat mtr = Mat::zeros(int(60/MapRes),int(60/MapRes),CV_8UC1);

    Point center = Point( origin_x+MapW/2, origin_y+int(20/MapRes) );
    double angle = theta; // in degrees
    double scale = 1;
    Mat rot_mat = getRotationMatrix2D( center, angle, scale );
    warpAffine( MapLog, MapLog, rot_mat, MapLog.size() );

    
    // BITWISE OR

    bitwise_or( m, MapLog, MapLog );

	// imshow("m2",m2);
	// waitKey(500);
	// imwrite("/home/hitvarth/Desktop/m2.png",m2);
	// destroyWindow("m2");

    imshow("map log", MapLog);
	waitKey(50);
	imwrite("/home/hitvarth/Desktop/MapLog.png",MapLog);
	// destroyWindow("map log");

 //    circle( m, Point2f(origin_x+MapW/2, origin_y+int(20/MapRes)), 20, Scalar( 255, 255, 255 ), 7, LINE_8 );
	// imshow("m", m);
	// waitKey(500);
	// imwrite("/home/hitvarth/Desktop/m.png",m);
	// destroyWindow("m");

	iter++;
}


int main(int argc, char **argv)
{
	ros::init(argc,argv,"MapLog_testing_node");
	ros::NodeHandle n;
	ros::Subscriber SubLane = n.subscribe<nav_msgs::OccupancyGrid>("/Lane_Occupancy_Grid",1,LaneCallback);
	// ros::Subscriber SubRoad = n.subscribe<nav_msgs::OccupancyGrid>("/Road_Occupancy_Grid",1,RoadCallback);
	ros::Rate RosLoopRate(20);     // 15 in RRT*
	
	// while(ros::ok())
	while(ros::ok())
	{
		ros::spinOnce();//check for incoming messages
		RosLoopRate.sleep(); 
	}
	destroyWindow("map log");
	destroyAllWindows();

return 0;
}


