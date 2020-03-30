///3D FRONT LINE FINDER
#ifndef PLANEFINDER_H
#define PLANEFINDER_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//#include "sensor_msgs/PointCloud2.h"
//#include "std_msgs/Empty.h"
//#include "vision_msgs/Skeletons.h"
//#include "justina_tools/JustinaTools.h"

// ROS headers
//#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL headers
#include <pcl/point_cloud.h>

#include "lineransac.h"

#include <stdint.h>
#include <math.h>

//Data structures

struct plane3D 
{
	cv::Mat pointCloud;

	bool debug;
	bool isplane;

	cv::Point3f point;
	cv::Point3f normal;

	cv::Mat plane;
	std::vector< cv::Point2i > indices;

	cv::Mat bigplane;
	std::vector<cv::Point> contour;
	std::vector< cv::Point2f > convexHullXY;
	std::vector< cv::Point2f > convexHullXZ;

	plane3D();
	~plane3D();
};
//******************

cv::Mat getNormals(cv::Mat pointCloud, int delta = 1);

plane3D getHorizontalPlane(cv::Mat pointCloud, cv::Mat mask, double maxDistPointToPlane = 0.02, int maxIterations = 1000, int minPointsForPlane = 10000);

plane3D getVerticalPlane(cv::Mat pointCloud, cv::Mat mask, double maxDistPointToPlane = 0.02, int maxIterations = 1000, int minPointsForPlane = 10000);

#endif
