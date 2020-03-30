// C++ standard headers
#include <exception>
#include <string>
//#include <math.h>
#include <cmath>

#include <vector>
#include <algorithm>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/topic.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// Erasers headers
#include <erasers_nav_msgs/GetSpaceCentroid.h>
#include <erasers_tools/transformations.h>
#include <erasers_tools/planefinder.h>

// OpenCV headers
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cv_bridge/cv_bridge.h>

// PCL headers
#include <pcl/point_cloud.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string rgbName         	= "RGB Source";
static const std::string depthName       	= "Depth Source";
static const std::string spaceName       	= "Space Segmentation";
static const std::string cameraFrame     	= "/xtion_rgb_optical_frame";
static const std::string imageTopic      	= "/hsrb/head_rgbd_sensor/rgb/image_rect_color";//"/xtion/rgb/image_raw";
static const std::string depthTopic      	= "/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw";//"/xtion/depth_registered/image_raw";
static const std::string cloudTopic      	= "/hsrb/head_rgbd_sensor/depth_registered/rectified_points"; //"/xtion/depth_registered/points";
static const std::string headStatusTopic  	= "/hsrb/head_trajectory_controller/state";
static const std::string jointStatesTopic 	= "/hsrb/joint_states";

//Global variables
double angle_yaw = 0, angle_pitch = 0;

//Head position wrt the robot in camera coordinates
double dx = 0.022, dy = 0.215, dz = 0.06;
double dy_torso = 0.752, dy_offset = 0.12;

bool debug = false;
int idcount = 0;

cv::RNG rng(12345);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Orientation transformation stuff
cv::Mat changeOrientation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud)
{
  cv::Mat pointCloud = cv::Mat::zeros(pclCloud->height, pclCloud->width, CV_32FC3);

  //Translation correction
  double COS = cos(angle_pitch);
  double SIN = sin(angle_pitch);

  double diff_z = dy * sin( fabs(angle_pitch) );
  double diff_y = dy * (1 - cos(fabs(angle_pitch)) );

  pcl::PointXYZRGB p;
  float tmp_y = 0, tmp_z = 0;

  //color_tmp.release();
  for (int h = 0; h < pointCloud.rows; h++)
  	for (int w = 0; w < pointCloud.cols; w++)
	{

		if ( !std::isnan(pclCloud->at(w, h).z) )
		{
			//Orientation correction
			p = pclCloud->at(w, h);
			tmp_y = p.y; tmp_z = p.z;
			p.y = tmp_y*COS - tmp_z*SIN;
			p.z = tmp_y*SIN + tmp_z*COS;

			//Position correction
			p.x = p.x + dx;
			p.y = dy_torso + dy - diff_y - p.y + dy_offset;
			p.z = p.z - dz + diff_z;

			pointCloud.at<cv::Vec3f>(h,w)[0] = p.x;
			pointCloud.at<cv::Vec3f>(h,w)[1] = p.y;
			pointCloud.at<cv::Vec3f>(h,w)[2] = p.z;
		}
	}

  return pointCloud;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Callback to obtain current head orientation
void headStatusCallback(const control_msgs::JointTrajectoryControllerState& hstatus)
{  
  trajectory_msgs::JointTrajectoryPoint actualstatus = hstatus.actual;
  
  angle_pitch = actualstatus.positions[0]; //tilt
  angle_yaw = actualstatus.positions[1]; //pan
}

// Updates the camera height
void jointStatesCallback(const sensor_msgs::JointState& jstate)
{  
  dy_torso = 0.752 + jstate.position[1] / 2; //arm_lift_joint
}

// ROS call back for every new image received
void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
  cv_bridge::CvImagePtr cvImgPtr;
  cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);

  if (debug)
  {
	cv::imshow(rgbName, cvImgPtr->image);
	cv::waitKey(15);
  }
}

// ROS call back for every new image received
void depthCallback(const sensor_msgs::ImageConstPtr& depthMsg)
{
  cv_bridge::CvImagePtr cvImgPtr;

  cvImgPtr = cv_bridge::toCvCopy(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1);
  cv::normalize(cvImgPtr->image, cvImgPtr->image, 1, 0, cv::NORM_MINMAX);

  if (debug)
  {
	cv::imshow(depthName, cvImgPtr->image);
	cv::waitKey(15);
  }
}

// OpenCV callback function for mouse events on a window
bool srvCallback(erasers_nav_msgs::GetSpaceCentroid::Request &req, erasers_nav_msgs::GetSpaceCentroid::Response &res)
{
  res.space.id = idcount++;
  res.space.isspace = false;

  //Spatial Bounding Box
  double width_1 = req.width_min, width_2 = req.width_max; //width or X axis
  double height_1 = req.height_min, height_2 = req.height_max; //height or Y axis
  double depth_1 = req.depth_min, depth_2 = req.depth_max; //depth or Z axis

  //Get Point Cloud Information
  ROS_INFO("Waiting for point cloud message... ");
  sensor_msgs::PointCloud2ConstPtr cloud = ros::topic::waitForMessage
      <sensor_msgs::PointCloud2>(cloudTopic);
  ROS_INFO("...point cloud message received. ");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud, *pclCloud);

  //Spatial clustering with kmeans
  cv::Mat hw_point = cv::Mat::zeros(1, 2, CV_32SC1);
  cv::Mat xyz_point = cv::Mat::zeros(1, 3, CV_32FC1);
  cv::Mat hw_samples;
  cv::Mat xyz_samples;

  //Orientation correction
  cv::Mat pointCloud = changeOrientation(pclCloud);

  //////////
  plane3D plane;
  //Plane segmentation

  //Normal extraction
  int blurSize = 5;

  cv::Mat pointCloudBlur;
  cv::blur(pointCloud, pointCloudBlur, cv::Size(blurSize, blurSize));
  cv::Mat normals = getNormals(pointCloud, 2);

  //Horizontal plane extraction
  double yThreshold = 0.8;
  cv::Mat  hNormals;
  cv::inRange( normals, cv::Scalar(-1.0, yThreshold, -1.0), cv::Scalar(1.0, 1.0, 1.0), hNormals);

  double delta_bb = 0.10;
  cv::Mat validPointCloud;
  cv::inRange(pointCloud, cv::Scalar(width_1, height_1-delta_bb, depth_1), 
	      cv::Scalar(width_2, height_2, depth_2), validPointCloud);
  hNormals = hNormals&validPointCloud;

  //cv::imshow("hNormals", hNormals);

  //Plane extraction with RANSAC
  double maxDistPointToPlane = 0.005;
  int maxIterations = 100;
  int minPointsForPlane = pointCloud.rows*pointCloud.cols*0.01;
  plane = getHorizontalPlane(pointCloud, hNormals, maxDistPointToPlane, maxIterations, minPointsForPlane);

  height_2 = height_2 - height_1;
  height_1 = plane.point.y + 0.01;
  height_2 = height_1 + height_2;

  if (cv::countNonZero(plane.plane) < 100)
  {
	std::cout << "space_finder_srv.empty" << std::endl;
	return true;
  }

  //int kernel_elem = 0; //0: Rect - 1: Cross - 2: Ellipse
  //int morph_size = 2; //kernel_size = 2*morph_size +1
  //cv::Mat element = cv::getStructuringElement( kernel_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );

  //cv::Mat plane_mask;
  //cv::morphologyEx( plane.bigplane, plane_mask, cv::MORPH_CLOSE, element );

  //if (debug)
  //{
	  //cv::imshow(planeName, plane.bigplane);
	  //cv::imshow("Closed Plane", plane_mask);
  //}

  cv::Mat dst;
  cv::cvtColor(plane.bigplane, dst, CV_GRAY2RGB);

  cv::Rect rect = cv::boundingRect(plane.contour);

  double minWidth = req.side_width;
  double minDepth = req.side_depth;

  int delta = 5;
  int dside = 10, wside = 10;
  //Front left side first
  cv::Point3f p_fl(0.0, 0.0, 0.0);
  for (int w = rect.x; w < rect.x + rect.width; w++)
  	for (int h = rect.y + rect.height - delta; h > rect.y; h--)
	{
		if(plane.bigplane.data[h*plane.bigplane.step + w] && 
		   plane.bigplane.data[(h-dside)*plane.bigplane.step + w] &&
		   plane.bigplane.data[(h-dside)*plane.bigplane.step + (w+wside)] &&
		   plane.bigplane.data[h*plane.bigplane.step + (w+wside)] )
		   {
			cv::Point3f p1 = plane.pointCloud.at<cv::Vec3f>((h-dside), w);
			cv::Point3f p2 = plane.pointCloud.at<cv::Vec3f>(h, w);
			cv::Point3f p3 = plane.pointCloud.at<cv::Vec3f>((h-dside), (w+wside));

			double ddist = std::sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.z-p2.z)*(p1.z-p2.z) ); 
			double wdist = std::sqrt( (p1.x-p3.x)*(p1.x-p3.x) + (p1.z-p3.z)*(p1.z-p3.z) ); 

			if ( ddist > minDepth && wdist > minWidth )
			{
				cv::Rect roi(w, (h-dside), wside, dside);
				int inliers = cv::countNonZero(plane.bigplane(roi));

				if ( inliers > 0.98*dside*wside )
				{
					cv::rectangle(dst, roi, cv::Scalar(0,0,255), 2);
					int n = 0;
					for (int j = h-dside; j < h; j++)
					  for (int i = w; i < w + wside; i++)
					  {
						cv::Point3f p = plane.pointCloud.at<cv::Vec3f>(j, i);
						if (p.z > 0.0)
						{
							p_fl = p_fl + p;
							n++;
						}
					  }

					p_fl = p_fl/n;
					res.space.isspace = true;

					w = plane.plane.cols; h = 0;

					if (debug)
					{
						std::cout << "space_finder_srv.frontleft: " << p_fl << std::endl;
						std::cout << "space_finder_srv.ditance: [" << wdist << ", " << ddist << "]" << std::endl;
						//std::cout << "space_finder_srv.side: " << side << std::endl;
					}
				}
			} 
			else
			{
				if (ddist < minDepth)
					dside++;

				if (wdist < minWidth)
					wside++;

				w = rect.x;
				h = rect.y + rect.height;
				
			}
		   }
	}

  dside = 10, wside = 10;
  //Front right side first
  cv::Point3f p_fr(0.0, 0.0, 0.0);
  for (int w = rect.x + rect.width; w > rect.x; w--)
  	for (int h = rect.y + rect.height - delta; h > rect.y; h--)
	{
		if(plane.bigplane.data[h*plane.bigplane.step + w] && 
		   plane.bigplane.data[(h-dside)*plane.bigplane.step + w] &&
		   plane.bigplane.data[(h-dside)*plane.bigplane.step + (w-wside)] &&
		   plane.bigplane.data[h*plane.bigplane.step + (w-wside)] )
		   {
			cv::Point3f p1 = plane.pointCloud.at<cv::Vec3f>((h-dside), w);
			cv::Point3f p2 = plane.pointCloud.at<cv::Vec3f>(h, w);
			cv::Point3f p3 = plane.pointCloud.at<cv::Vec3f>((h-dside), (w-wside));

			double ddist = std::sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.z-p2.z)*(p1.z-p2.z) ); 
			double wdist = std::sqrt( (p1.x-p3.x)*(p1.x-p3.x) + (p1.z-p3.z)*(p1.z-p3.z) ); 

			if ( ddist > minDepth && wdist > minWidth )
			{
				cv::Rect roi((w-wside), (h-dside), wside, dside);
				int inliers = cv::countNonZero(plane.bigplane(roi));

				if ( inliers > 0.98*wside*dside )
				{
					cv::rectangle(dst, roi, cv::Scalar(0,255,0), 2);
					int n = 0;
					for (int j = h-dside; j < h; j++)
					  for (int i = w-wside; i < w; i++)
					  {
						cv::Point3f p = plane.pointCloud.at<cv::Vec3f>(j, i);
						if (p.z > 0.0)
						{
							p_fr = p_fr + p;
							n++;
						}
					  }

					p_fr = p_fr/n;
					res.space.isspace = true;

					w = 0; h = 0;

					if (debug)
					{
						std::cout << "space_finder_srv.frontright: " << p_fr << std::endl;
						std::cout << "space_finder_srv.ditance: [" << wdist << ", " << ddist << "]" << std::endl;
						//std::cout << "space_finder_srv.ditance: " << dist << std::endl;
						//std::cout << "space_finder_srv.side: " << side << std::endl;
					}
				}
			} 
			else
			{
				if (ddist < minDepth)
					dside++;

				if (wdist < minWidth)
					wside++;

				w = rect.x + rect.width;
				h = rect.y + rect.height;
			}
		   }
	}

  dside = 10, wside = 10;
  //Back left side first
  cv::Point3f p_bl(0.0, 0.0, 0.0);
  for (int h = rect.y + delta; h < rect.y + rect.height; h++)
  	for (int w = rect.x; w < rect.x + rect.width; w++)
	{
		if(plane.bigplane.data[h*plane.bigplane.step + w] && 
		   plane.bigplane.data[(h+dside)*plane.bigplane.step + w] &&
		   plane.bigplane.data[(h+dside)*plane.bigplane.step + (w+wside)] &&
		   plane.bigplane.data[h*plane.bigplane.step + (w+wside)] )
		   {
			cv::Point3f p1 = plane.pointCloud.at<cv::Vec3f>(h, w);
			cv::Point3f p2 = plane.pointCloud.at<cv::Vec3f>((h+dside), w);
			cv::Point3f p3 = plane.pointCloud.at<cv::Vec3f>(h, (w+wside));

			double ddist = std::sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.z-p2.z)*(p1.z-p2.z) ); 
			double wdist = std::sqrt( (p1.x-p3.x)*(p1.x-p3.x) + (p1.z-p3.z)*(p1.z-p3.z) ); 

			if ( ddist > minDepth && wdist > minWidth )
			{
				cv::Rect roi(w, h, wside, dside);
				int inliers = cv::countNonZero(plane.bigplane(roi));

				if ( inliers > 0.98*wside*dside )
				{
					cv::rectangle(dst, roi, cv::Scalar(255,0,0), 2);
					int n = 0;
					for (int j = h; j < h + dside; j++)
					  for (int i = w; i < w + wside; i++)
					  {
						cv::Point3f p = plane.pointCloud.at<cv::Vec3f>(j, i);
						if (p.z > 0.0)
						{
							p_bl = p_bl + p;
							n++;
						}
					  }

					p_bl = p_bl/n;
					res.space.isspace = true;

					w = plane.plane.cols; h = plane.plane.rows;

					if (debug)
					{
						std::cout << "space_finder_srv.backleft: " << p_bl << std::endl;
						std::cout << "space_finder_srv.ditance: [" << wdist << ", " << ddist << "]" << std::endl;
						//std::cout << "space_finder_srv.ditance: " << dist << std::endl;
						//std::cout << "space_finder_srv.side: " << side << std::endl;
					}
				}
			} 
			else
			{
				if (ddist < minDepth)
					dside++;

				if (wdist < minWidth)
					wside++;

				w = rect.x;
				h = rect.y + delta;
			}
		   }
	}

  dside = 10, wside = 10;
  //Back right side first
  cv::Point3f p_br(0.0, 0.0, 0.0);
  for (int h = rect.y + delta; h < rect.y + rect.height; h++)
  	for (int w = rect.x + rect.width; w > rect.x; w--)
	{
		if(plane.bigplane.data[h*plane.bigplane.step + w] && 
		   plane.bigplane.data[(h+dside)*plane.bigplane.step + w] &&
		   plane.bigplane.data[(h+dside)*plane.bigplane.step + (w-wside)] &&
		   plane.bigplane.data[h*plane.bigplane.step + (w-wside)] )
		   {
			cv::Point3f p1 = plane.pointCloud.at<cv::Vec3f>(h, w);
			cv::Point3f p2 = plane.pointCloud.at<cv::Vec3f>((h+dside), w);
			cv::Point3f p3 = plane.pointCloud.at<cv::Vec3f>(h, (w-wside));

			double ddist = std::sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.z-p2.z)*(p1.z-p2.z) ); 
			double wdist = std::sqrt( (p1.x-p3.x)*(p1.x-p3.x) + (p1.z-p3.z)*(p1.z-p3.z) ); 

			if ( ddist > minDepth && wdist > minWidth )
			{
				cv::Rect roi((w-wside), h, wside, dside);
				int inliers = cv::countNonZero(plane.bigplane(roi));

				if ( inliers > 0.98*wside*dside )
				{
					cv::rectangle(dst, roi, cv::Scalar(0,255,255), 2);
					int n = 0;
					for (int j = h; j < h + dside; j++)
					  for (int i = w - wside; i < w; i++)
					  {
						cv::Point3f p = plane.pointCloud.at<cv::Vec3f>(j, i);
						if (p.z > 0.0)
						{
							p_br = p_br + p;
							n++;
						}
					  }

					p_br = p_br/n;
					res.space.isspace = true;

					w = 0; h = plane.plane.rows;

					if (debug)
					{
						std::cout << "space_finder_srv.backright: " << p_br << std::endl;
						std::cout << "space_finder_srv.ditance: [" << wdist << ", " << ddist << "]" << std::endl;
						//std::cout << "space_finder_srv.ditance: " << dist << std::endl;
						//std::cout << "space_finder_srv.side: " << side << std::endl;
					}
				}
			} 
			else
			{
				if (ddist < minDepth)
					dside++;

				if (wdist < minWidth)
					wside++;

				w = rect.x + rect.width;
				h = rect.y + delta;
			}
		   }
	}

  if (debug)
  {
	cv::imshow(spaceName, dst);
  }

  if (!res.space.isspace)
  {
	std::cout << "space_finder_srv.empty" << std::endl;
	return true;
  }

  std::vector<float_t> vec_fl, vec_fr, vec_bl, vec_br;

  vec_fl.push_back((float_t)p_fl.x);
  vec_fl.push_back((float_t)p_fl.y);
  vec_fl.push_back((float_t)p_fl.z);

  vec_fr.push_back((float_t)p_fr.x);
  vec_fr.push_back((float_t)p_fr.y);
  vec_fr.push_back((float_t)p_fr.z);

  vec_bl.push_back((float_t)p_bl.x);
  vec_bl.push_back((float_t)p_bl.y);
  vec_bl.push_back((float_t)p_bl.z);

  vec_br.push_back((float_t)p_br.x);
  vec_br.push_back((float_t)p_br.y);
  vec_br.push_back((float_t)p_br.z);

  res.space.frontleft = vec_fl;
  res.space.frontright = vec_fr;
  res.space.backleft = vec_bl;
  res.space.backright = vec_br;

  std::cout << "space_finder_srv.space" << std::endl;
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "space_finder_srv");

  ROS_INFO("Starting space_finder_srv application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh, nh2, nh3, nh4, nh5, nh6;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  for (int i = 0; i < argc; i++)
  {
	  if ( strcmp( argv[i], "-debug") == 0 )
		  debug = true;
  }

  if (debug)
  {
	  // Create the window to show HSR camera images
	  cv::namedWindow(rgbName, cv::WINDOW_AUTOSIZE);
	  cv::namedWindow(depthName, cv::WINDOW_AUTOSIZE);
	  cv::namedWindow(spaceName, cv::WINDOW_AUTOSIZE);
  }

  ros::ServiceServer service = nh.advertiseService("/erasers/navigation/space_finder_srv", srvCallback);

  ROS_INFO_STREAM("Subscribing to " << headStatusTopic << " ...");
  ros::Subscriber subhead = nh2.subscribe(headStatusTopic, 1, headStatusCallback);

  // Define ROS topic from where HSR publishes images
  image_transport::ImageTransport it(nh3);
  // use compressed image transport to use less network bandwidth
  image_transport::TransportHints transportHint("compressed");

  ROS_INFO_STREAM("Subscribing to " << imageTopic << " ...");
  image_transport::Subscriber subrgb = it.subscribe(imageTopic, 1, imageCallback, transportHint);

  ROS_INFO_STREAM("Subscribing to " << depthTopic << " ...");
  ros::Subscriber subdepth = nh4.subscribe(depthTopic, 1, depthCallback);

//  ROS_INFO_STREAM("Subscribing to " << cloudTopic << " ...");
//  ros::Subscriber subcloud = nh5.subscribe(cloudTopic, 1, cloudCallback);

  ROS_INFO_STREAM("Subscribing to " << jointStatesTopic << " ...");
  ros::Subscriber subjoint = nh6.subscribe(jointStatesTopic, 1, jointStatesCallback);

  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::spin();

  if (debug)
  {
	  cv::destroyWindow(rgbName);
	  cv::destroyWindow(depthName);
	  cv::destroyWindow(spaceName);
  }

  return EXIT_SUCCESS;
}
