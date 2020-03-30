// C++ standard headers
#include <exception>
#include <string>
#include <math.h>
#include <iostream>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
//#include <image_transport/image_transport.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <ros/topic.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// PCL headers
#include <pcl/PCLHeader.h>
#include <pcl/point_cloud.h>

// Erasers headers
#include <erasers_nav_msgs/ProjectedView.h>
//#include <erasers_nav_msgs/GetProjectedView.h>
#include <erasers_nav_msgs/GetLinePoint.h>

#include "erasers_tools/linefinder.h"

#include <erasers_tools/transformations.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string rgbName          = "Line_Server_RGB";
static const std::string povName          = "Line_Server_POV";
static const std::string cloudTopic       = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points"; //"/xtion/depth_registered/points";
static const std::string headStatusTopic  = "/hsrb/head_trajectory_controller/state";//"/head_controller/state";
static const std::string jointStatesTopic = "/hsrb/joint_states";

//pcl Point Cloud
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

double angle_yaw = 0, angle_pitch = 0;
//double angle_pan = 0, angle_tilt = 0;

double dx = 0.06, dy = 0.022, dz = 0.215;
double dz_torso = 0.752, dz_offset = 0.13;//0.08;

//Change of perspective stuff
pointOfViewParameters povParams;

//Global parameters
bool debug = false;
int count = 0;

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

std_msgs::Header fromPCL(const pcl::PCLHeader &pcl_header)
{
  std_msgs::Header header;

  header.stamp.fromNSec(pcl_header.stamp * 1000ull);
  header.seq = pcl_header.seq;
  header.frame_id = pcl_header.frame_id;

  return header;
}

void headStatusCallback(const control_msgs::JointTrajectoryControllerState& hstatus)
{  
  trajectory_msgs::JointTrajectoryPoint actualstatus = hstatus.actual;
  
  angle_pitch = actualstatus.positions[0]; //tilt
  angle_yaw = actualstatus.positions[1]; //pan
}

void jointStatesCallback(const sensor_msgs::JointState& jstate)
{  
  dz_torso = 0.752 + jstate.position[1] / 2; //arm_lift_joint

  //std::cout << "\ndz_torso: " << dz_torso << "\n" << std::endl;
}

/*void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{  
  if ( (cloud->width * cloud->height) == 0)
    return;

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud, *pclCloud);
}*/

bool srvCallback(erasers_nav_msgs::GetLinePoint::Request &req, erasers_nav_msgs::GetLinePoint::Response &res)
{
  res.line_point.id = count++;
  res.line_point.isline = false;

  ROS_INFO("Waiting for point cloud message... ");
  sensor_msgs::PointCloud2ConstPtr cloud = ros::topic::waitForMessage
      <sensor_msgs::PointCloud2>(cloudTopic);
  ROS_INFO("...point cloud message received. ");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud, *pclCloud);

  double diff_x = dz * sin( fabs(angle_pitch) );
  double diff_z = dz * (1 - cos(fabs(angle_pitch)) );

  povParams.angle = (-1)*angle_pitch * 180 / 3.14159265358979323846;//headTilt * 180 / 3.14159265358979323846; //Camera pitch angle in degrees
  //std::cout << "change_of_perspective.->Changing perspective" << std::endl;

  //Change height value to head coordinates 
  double tmpmaxx = povParams.areaLimits.maxX; 
  double tmpminy = povParams.areaLimits.minY;
  double tmpmaxy = povParams.areaLimits.maxY;
  double tmpmaxz = povParams.areaLimits.maxZ; 

  if (req.maxx > 0)
  	povParams.areaLimits.maxX = req.maxx;

  if (req.maxz > 0)
  	povParams.areaLimits.maxZ = req.maxz;

  if (req.miny > 0 && req.maxy > 0 && req.miny < req.maxy)
  {
  	povParams.areaLimits.minY = req.miny;
  	povParams.areaLimits.maxY = req.maxy;
  }

  double miny = dz_torso + dz + dz_offset - diff_z - povParams.areaLimits.maxY;
  double maxy = dz_torso + dz + dz_offset - diff_z - povParams.areaLimits.minY;

  povParams.areaLimits.minY = miny;
  povParams.areaLimits.maxY = maxy;

  changeViewPerspective ( pclCloud, povParams);
  //std::cout << "change_of_perspective.->Perspective changed" << std::endl;

  povParams.areaLimits.maxX = tmpmaxx;
  povParams.areaLimits.minY = tmpminy;
  povParams.areaLimits.maxY = tmpmaxy;
  povParams.areaLimits.maxZ = tmpmaxz;

  bool dist3d = true; //Calculate the line equation in pixel [FALSE ] or metric [TRUE] units

  cv::Mat line = frontLine(povParams, dist3d);

  if ( line.empty() )
  {
	std::cout << "front_line_srv.empty" << std::endl;
	return true;
  }

  cv::Mat metric_start, metric_end, pixel_start, pixel_end;

  metric_start.push_back(getCloudPoint(line.row(0)));
  metric_start.convertTo(metric_start, CV_32F);

  metric_start.at<float>(0) = metric_start.at<float>(0) - float(dx) + float(diff_x);
  metric_start.at<float>(1) = metric_start.at<float>(1) - float(dy);
  metric_start.at<float>(2) = dz_torso + dz  - float(diff_z) - metric_start.at<float>(2) + dz_offset;
  //metric_start.at<float>(2) = float(dz_torso + dz);

  metric_end.push_back(getCloudPoint(line.row(1)));
  metric_end.convertTo(metric_end, CV_32F);

  metric_end.at<float>(0) = metric_end.at<float>(0) - float(dx) + float(diff_x);
  metric_end.at<float>(1) = metric_end.at<float>(1) - float(dy);
  metric_end.at<float>(2) = dz_torso + dz  - float(diff_z) - metric_end.at<float>(2) + dz_offset;
  //metric_end.at<float>(2) = metric_end.at<float>(2) + float(diff_z);

  if (dist3d)
  {
    pixel_start.push_back(line.row(2));
    pixel_start.convertTo(pixel_start, CV_32F);

    pixel_end.push_back(line.row(3));
    pixel_end.convertTo(pixel_end, CV_32F);
  }
  else
  {
    pixel_start = cv::Mat::zeros(1, 3, CV_32F);
    pixel_end = cv::Mat::zeros(1, 3, CV_32F);
  }

  ///Send messages
  std::vector<float_t> vec_metric_start, vec_metric_end, vec_pixel_start, vec_pixel_end;
  int start_w, start_h, end_w, end_h;

  res.line_point.isline = true;

  Transformations::CvMatf_ToVectMsg(metric_start, &vec_metric_start, start_w, start_h);
  Transformations::CvMatf_ToVectMsg(pixel_start, &vec_pixel_start, start_w, start_h);

  Transformations::CvMatf_ToVectMsg(metric_end, &vec_metric_end, end_w, end_h);
  Transformations::CvMatf_ToVectMsg(pixel_end, &vec_pixel_end, end_w, end_h);

  res.line_point.header = fromPCL(pclCloud->header);

  res.line_point.metric_start = vec_metric_start;
  res.line_point.pixel_start = vec_pixel_start;
  res.line_point.start_w = start_w;
  res.line_point.start_h = start_h;
 
  res.line_point.metric_end = vec_metric_end;
  res.line_point.pixel_end = vec_pixel_end;
  res.line_point.end_w = end_w;
  res.line_point.end_h = end_h;

  if (debug)
  {
	std::cout << "front_line_srv.->X1: " << metric_start.at<float>(0) << ", Y1: " << metric_start.at<float>(1) << ", Z1: " << metric_start.at<float>(2) << std::endl;
	std::cout << "front_line_srv.->X2: " << metric_end.at<float>(0) << ", Y2: " << metric_end.at<float>(1) << ", Z2: " << metric_end.at<float>(2) << std::endl;


	if (dist3d)
	{
	   std::cout << "front_line_srv.->P1X: " << (int)pixel_start.at<float>(0) << ", P1Y: " << (int)pixel_start.at<float>(1) << std::endl;
	   std::cout << "front_line_srv.->P2X: " << (int)pixel_end.at<float>(0) << ", P2Y: " << (int)pixel_end.at<float>(1) << std::endl;

	   cv::Point pt1 = cv::Point((int)pixel_start.at<float>(0),(int)pixel_start.at<float>(1));
	   cv::Point pt2 = cv::Point((int)pixel_end.at<float>(0),(int)pixel_end.at<float>(1));

	   //Get extreme points
	   cv::Scalar blue = cv::Scalar(255,0,0);
	   int thickness = 2;
	   int connectivity = 8;
	   cv::line(povParams.src, pt1, pt2, blue, thickness, connectivity);
	}

	cv::imshow(rgbName, povParams.src);
	cv::imshow(povName, povParams.front);

	cv::waitKey(15);
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "front_line_srv");

  ROS_INFO("Starting front_line_srv application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh, nh2, nh3, nh4;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  char *filename; 
  filename = "path/filename.yaml";

  for (int i = 0; i < argc; i++)
  {
	  if ( strcmp( argv[i], "-debug") == 0 )
		  debug = true;

	  if ( strcmp( argv[i], "-conf") == 0 )
	  {
		  filename = "";
		  filename = strdup( argv[++i] );
		  sprintf ( filename, "%s", filename );
	  }
  }

  cv::FileStorage fs;
  fs.open(filename, cv::FileStorage::READ);

  if (!fs.isOpened())
  {
	  std::cerr << "Failed to open " << filename << std::endl;
	  return 1;
  }

  povParams.color = static_cast<int>(fs["color"]) != 0; //Color or BW debug image
  povParams.fullData = static_cast<int>(fs["fullData"]) != 0; //If false, then use a limited height range

  //Define 3D search boinding box in front of the camera
  fs["maxX"] >> povParams.areaLimits.maxX; //Width
  fs["maxZ"] >> povParams.areaLimits.maxZ; // Depth
  fs["minY"] >> povParams.areaLimits.minY; //Min height in robot coordinates
  fs["maxY"] >> povParams.areaLimits.maxY; //Max height in robot coordinates

  fs.release();


  // Create the window to show TIAGo's camera images
  if (debug)
  {
	cv::namedWindow(rgbName, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(povName, cv::WINDOW_AUTOSIZE);
  }

  ros::ServiceServer service = nh.advertiseService("/erasers/navigation/front_line_srv", srvCallback);

  ROS_INFO_STREAM("Subscribing to " << headStatusTopic << " ...");
  ros::Subscriber subhead = nh2.subscribe(headStatusTopic, 1, headStatusCallback);

  ROS_INFO_STREAM("Subscribing to " << jointStatesTopic << " ...");
  ros::Subscriber subjoint = nh3.subscribe(jointStatesTopic, 1, jointStatesCallback);
 
  //ROS_INFO_STREAM("Subscribing to " << cloudTopic << " ...");
  //ros::Subscriber subcloud = nh4.subscribe(cloudTopic, 1, cloudCallback);
 
  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::Rate loop_rate(5);
  
  ros::spin();

  return EXIT_SUCCESS;
}
