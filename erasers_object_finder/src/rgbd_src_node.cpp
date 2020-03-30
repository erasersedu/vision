// C++ standard headers
#include <exception>
#include <string>
//#include <math.h>
#include <cmath>

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

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// PCL headers
#include <pcl/point_cloud.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string rgbName         	= "RGB Source";
static const std::string depthName       	= "Depth Source";
static const std::string planeName       	= "Plane Segmentation";
static const std::string cameraFrame     	= "/xtion_rgb_optical_frame";
static const std::string imageTopic      	= "/hsrb/head_rgbd_sensor/rgb/image_rect_color";//"/xtion/rgb/image_raw";
static const std::string depthTopic      	= "/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw";//"/xtion/depth_registered/image_raw";
static const std::string cameraInfoTopic 	= "/hsrb/head_rgbd_sensor/rgb/camera_info";//"/xtion/rgb/camera_info";
static const std::string cloudTopic      	= "/hsrb/head_rgbd_sensor/depth_registered/rectified_points"; //"/xtion/depth_registered/points";
static const std::string headStatusTopic  	= "/hsrb/head_trajectory_controller/state";
static const std::string jointStatesTopic 	= "/hsrb/joint_states";

// Intrinsic parameters of the camera
cv::Mat cameraIntrinsics;

//Global variables
double angle_yaw = 0, angle_pitch = 0;

//double dx = 0.06, dy = 0.022, dz = 0.215;
//double dz_torso = 0.752, dz_offset = 0.08;

double dx = 0.022, dy = 0.215, dz = 0.06;
double dy_torso = 0.752, dy_offset = 0.08;

//pcl Point Cloud
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

ros::Time latestImageStamp;
ros::Time latestDepthStamp;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define ERROR_APROX_METRIC 0.025

//Random sampling n numbers out maxn
cv::Mat randomSample(int n, cv::Mat points)
{
	cv::Mat sample;
	std::vector<int> rand_x;

	int x;
	bool isReg;

	int H = points.rows;

	for(int i = 0; i < n; i++ )
	{
		do{
			isReg = false;
			x = rand() % H;	
			for(int j = 0 ;j < int(rand_x.size()); j++)
			{
				if(x == rand_x[j]){
					isReg=true;
					break;
				}
			}
		}
		while( isReg );

		rand_x.push_back(x);
	}

	for (int i = 0; i < n; i++)
		sample.push_back(points.row(rand_x[i]));

	return sample;
};

//cv::Mat color_tmp, bgr;
//Get the inliers in a point set over the plane 
//defined by a unit normal (row(0)) and a point on plane (row(1))
std::vector<int> findConsensus(cv::Mat points, cv::Mat plane)
{
	std::vector<int> consensus;

	int H = points.rows;

	cv::Mat n = plane.row(0);
	cv::Mat p = plane.row(1);

	double distance;
	double errorAprox = ERROR_APROX_METRIC;
	for(int i = 0; i < H; i++ )
	{
		//Get point to plane (defined by a unit norman and a point) distance
		cv::Mat p0 = points.row(i);

		cv::Mat d = p - p0;

		distance = n.dot(d);

		if( fabs(distance) <=  errorAprox )
			consensus.push_back(i);
	}

	return consensus;
};

//Get the best line that fits a point set using RANSAC
cv::Mat planeRANSAC(cv::Mat points)
{

	//Initialize random number generator 
	srand( (unsigned int) time(NULL) );

	int nloop = 0;
	
	cv::Mat best_sample;

	std::vector<int> consensus;
	int myConsensus = 0, maxConsensus = 0;
	int minConsensus = (int)(0.51*points.rows); //51% data points

	double averageInliers = 0;

	bool found = false;
	while( nloop++ < 20 )
	{
		//Select random point in the plane
		cv::Mat point = randomSample(1, points);

		//Define normal to the plane
		cv::Mat normal = cv::Mat::zeros(1, 3, CV_64FC1);
		normal.at<double>(0) = 0.0;
		normal.at<double>(1) = 0.0;
		normal.at<double>(2) = 1.0;

		cv::Mat plane;
		plane.push_back(normal);
		plane.push_back(point);

		//std::vector<int> consensus;
		consensus.clear();
		consensus = findConsensus(points, plane); //3D line

		myConsensus = consensus.size();

		if( myConsensus > maxConsensus )
		{
			best_sample.release();
			best_sample.push_back(normal);
			best_sample.push_back(point);

			maxConsensus = myConsensus;
			averageInliers = (double) maxConsensus / points.rows;
		}

		if( maxConsensus >= minConsensus )
			found = true;
	}

	/*for (int i = 0; i < consensus.size(); i++)
	{
		int w = (int)color_tmp.at<double>(i,3);
		int h = (int)color_tmp.at<double>(i,4);

		bgr.data[h*bgr.step + w*3] = (unsigned char)(color_tmp.at<double>(i,0)/2);
		bgr.data[h*bgr.step + w*3 + 1] = (unsigned char)((color_tmp.at<double>(i,1)+255)/2);
		bgr.data[h*bgr.step + w*3 + 2] = (unsigned char)(color_tmp.at<double>(i,2)/2);
	}*/

	ROS_INFO_STREAM( "Average inliers: " << maxConsensus << "/" << points.rows << " = " << averageInliers );

	return best_sample;
};

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

  //std::cout << "\ndz_torso: " << dz_torso << "\n" << std::endl;
}

// ROS call back for every new image received
void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
  latestImageStamp = imgMsg->header.stamp;

  cv_bridge::CvImagePtr cvImgPtr;

  cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
  cv::imshow(rgbName, cvImgPtr->image);
  cv::waitKey(15);
}

// ROS call back for every new image received
void depthCallback(const sensor_msgs::ImageConstPtr& depthMsg)
{
  latestDepthStamp = depthMsg->header.stamp;
  cv_bridge::CvImagePtr cvImgPtr;

  cvImgPtr = cv_bridge::toCvCopy(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1);
  cv::normalize(cvImgPtr->image, cvImgPtr->image, 1, 0, cv::NORM_MINMAX);
  //ROS_INFO_STREAM("Matrix type: " << type2str(cvImgPtr->image.type()));

  cv::imshow(depthName, cvImgPtr->image);
  cv::waitKey(15);
}

/*void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{  
  if ( (cloud->width * cloud->height) == 0)
    return;

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud, *pclCloud);
}*/

// OpenCV callback function for mouse events on a window
void onMouseRGB( int event, int u, int v, int, void* )
{
  if ( event != cv::EVENT_LBUTTONDOWN )
      return;

  geometry_msgs::PointStamped pointStamped;

  pointStamped.header.frame_id = cameraFrame;
  pointStamped.header.stamp    = latestImageStamp;

  //compute normalized coordinates of the selected pixel
  double x = ( u  - cameraIntrinsics.at<double>(0,2) )/ cameraIntrinsics.at<double>(0,0);
  double y = ( v  - cameraIntrinsics.at<double>(1,2) )/ cameraIntrinsics.at<double>(1,1);
  double Z = 1.0; //define an arbitrary distance

  ROS_INFO_STREAM("Normalized coordinates at pixel (" << u << ", " << v << ") are ( " << 
                   x*Z << ", " << y*Z << ", " << Z << " )");

  pointStamped.point.x = x * Z;
  pointStamped.point.y = y * Z;
  pointStamped.point.z = Z;   
}

// OpenCV callback function for mouse events on a window
void onMouseDepth( int event, int u, int v, int, void* )
{
  if ( event != cv::EVENT_LBUTTONDOWN )
      return;

  ROS_INFO_STREAM("Pitch angle: " << (-1)*angle_pitch * 180 / 3.14159265358979323846);

  ROS_INFO("Waiting for point cloud message... ");
  sensor_msgs::PointCloud2ConstPtr cloud = ros::topic::waitForMessage
      <sensor_msgs::PointCloud2>(cloudTopic, ros::Duration(10.0));

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud, *pclCloud);

  //pcl::PointXYZRGB p = pclCloud->at(u,v);

  //if (std::isnan(p.z))
  //{
  //	  p.x = 0.0; p.y = 0.0; p.z = 0.0;
  //}

  cv::Mat points;
  cv::Mat bgr = cv::Mat::zeros(pclCloud->height, pclCloud->width, CV_8UC3);
  //bgr = cv::Mat::zeros(pclCloud->height, pclCloud->width, CV_8UC3);

  //ROS_INFO_STREAM( "(h, w): (" << pclCloud->height << ", " << pclCloud->width << ")" );

  //ROS_INFO_STREAM("RBG values at pixel (" << u << ", " << v << ") are (" << 
  //                 (int)p.r << ", " << (int)p.g << ", " << (int)p.b << ")" );

  //ROS_INFO_STREAM("XYZ coordinates at pixel (" << u << ", " << v << ") are (" << 
  //                 (double)p.x << ", " << (double)p.y << ", " << (double)p.z << ")" );

  int kernel_size = 5;
  cv::Vec3d v1, v2;
  pcl::PointXYZRGB p, p1, p2;
  double tmp_y = 0, tmp_z = 0;

  double COS = cos(angle_pitch);
  double SIN = sin(angle_pitch);

  //double diff_x = dz * sin( fabs(angle_pitch) );
  //double diff_z = dz * (1 - cos(fabs(angle_pitch)) );

  double diff_z = dy * sin( fabs(angle_pitch) );
  double diff_y = dy * (1 - cos(fabs(angle_pitch)) );

  //color_tmp.release();
  for (int h=0+kernel_size; h<bgr.rows-kernel_size; h++)
  	for (int w=0+kernel_size; w<bgr.cols-kernel_size; w++)
	{

		if ( std::isnan(pclCloud->at(w, h).z) || 
		     std::isnan(pclCloud->at(w-kernel_size, h-kernel_size).z) || 
		     std::isnan(pclCloud->at(w+kernel_size, h+kernel_size).z) || 
		     std::isnan(pclCloud->at(w-kernel_size, h+kernel_size).z) ||
		     std::isnan(pclCloud->at(w+kernel_size, h-kernel_size).z) )
		{
			bgr.data[h*bgr.step + w*3] = (unsigned char)0;
			bgr.data[h*bgr.step + w*3 + 1] = (unsigned char)0;
			bgr.data[h*bgr.step + w*3 + 2] = (unsigned char)0;
		} else
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

			if (p.y > 0.05)
			{
				//Orientation correction
				p1 = pclCloud->at(w-kernel_size, h-kernel_size);
				tmp_y = p1.y; tmp_z = p1.z;
				p1.y = tmp_y*COS - tmp_z*SIN;
				p1.z = tmp_y*SIN + tmp_z*COS;

				//Position correction
				p1.x = p1.x + dx;
				p1.y = dy_torso + dy - diff_y - p1.y + dy_offset;
				p1.z = p1.z - dz + diff_z;

				//Orientation correction
				p2 = pclCloud->at(w+kernel_size, h+kernel_size);
				tmp_y = p2.y; tmp_z = p2.z;
				p2.y = tmp_y*COS - tmp_z*SIN;
				p2.z = tmp_y*SIN + tmp_z*COS;

				//Position correction
				p2.x = p2.x + dx;
				p2.y = dy_torso + dy - diff_y - p2.y + dy_offset;
				p2.z = p2.z - dz + diff_z;

				v1[0] = (double)p2.x-p1.x;
				v1[1] = (double)p2.y-p1.y;
				v1[2] = (double)p2.z-p1.z;

				//Orientation correction
				p1 = pclCloud->at(w+kernel_size, h-kernel_size);
				tmp_y = p1.y; tmp_z = p1.z;
				p1.y = tmp_y*COS - tmp_z*SIN;
				p1.z = tmp_y*SIN + tmp_z*COS;

				//Position correction
				p1.x = p1.x + dx;
				p1.y = dy_torso + dy - diff_y - p1.y + dy_offset;
				p1.z = p1.z - dz + diff_z;

				//Orientation correction
				p2 = pclCloud->at(w-kernel_size, h+kernel_size);
				tmp_y = p2.y; tmp_z = p2.z;
				p2.y = tmp_y*COS - tmp_z*SIN;
				p2.z = tmp_y*SIN + tmp_z*COS;

				//Position correction
				p2.x = p2.x + dx;
				p2.y = dy_torso + dy - diff_y - p2.y + dy_offset;
				p2.z = p2.z - dz + diff_z;

				v2[0] = (double)p2.x-p1.x;
				v2[1] = (double)p2.y-p1.y;
				v2[2] = (double)p2.z-p1.z;

				cv::Vec3d v_cross = v1.cross(v2);

				//double norm_y = cos(fabs(angle_pitch));
				//double norm_z = sin(fabs(angle_pitch));
				//double diff = pow(norm_y - fabs(v_cross[1]/norm(v_cross)) ,2) + pow(norm_z - fabs(v_cross[2]/norm(v_cross)), 2);
				//diff = sqrt(diff);

				//Define normal to the plane
				cv::Mat normal = cv::Mat::zeros(1, 3, CV_64FC1);
				normal.at<double>(0) = 0.0;
				normal.at<double>(1) = 0.0;
				normal.at<double>(2) = 1.0;

				double projection = normal.at<double>(0)*v_cross[0]/norm(v_cross) +
						    normal.at<double>(1)*v_cross[1]/norm(v_cross) +
						    normal.at<double>(2)*v_cross[2]/norm(v_cross);

				double diff = 1.0 - fabs(projection);

				if ( fabs(diff) < 0.1 )
				{
					cv::Mat point = cv::Mat::zeros(1, 3, CV_64FC1);
					point.at<double>(0) = (double)p.x;
					point.at<double>(1) = (double)p.y;
					point.at<double>(2) = (double)p.z;
					points.push_back(point);

					/*cv::Mat color_p = cv::Mat::zeros(1, 5, CV_64FC1);
					color_p.at<double>(0) = (double)p.b;
					color_p.at<double>(1) = (double)p.g;
					color_p.at<double>(2) = (double)p.r;
					color_p.at<double>(3) = (double)w;
					color_p.at<double>(4) = (double)h;
					color_tmp.push_back(color_p);*/

					bgr.data[h*bgr.step + w*3] = (unsigned char)((p.b+255)/2);
					bgr.data[h*bgr.step + w*3 + 1] = (unsigned char)(p.g/2);
					bgr.data[h*bgr.step + w*3 + 2] = (unsigned char)(p.r/2);
				} else
				{
					bgr.data[h*bgr.step + w*3] = (unsigned char)p.b;
					bgr.data[h*bgr.step + w*3 + 1] = (unsigned char)p.g;
					bgr.data[h*bgr.step + w*3 + 2] = (unsigned char)p.r;
				}
			} else
			{
				bgr.data[h*bgr.step + w*3] = (unsigned char)(p.b/2);
				bgr.data[h*bgr.step + w*3 + 1] = (unsigned char)(p.g/2);
				bgr.data[h*bgr.step + w*3 + 2] = (unsigned char)((p.r+255)/2);
			}
		}
	}

  //ROS_INFO_STREAM( "Points: " << points.rows );
  //planeRANSAC(points);

  cv::imshow(planeName, bgr);
  cv::waitKey(15);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "rgbd_src_node");

  ROS_INFO("Starting rgbd_src_node application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh, nh2, nh3, nh4, nh5;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Get the camera intrinsic parameters from the appropriate ROS topic
  ROS_INFO("Waiting for camera intrinsics ... ");
  sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage
      <sensor_msgs::CameraInfo>(cameraInfoTopic, ros::Duration(10.0));
  if(msg.use_count() > 0)
  {
    cameraIntrinsics = cv::Mat::zeros(3,3,CV_64F);
    cameraIntrinsics.at<double>(0, 0) = msg->K[0]; //fx
    cameraIntrinsics.at<double>(1, 1) = msg->K[4]; //fy
    cameraIntrinsics.at<double>(0, 2) = msg->K[2]; //cx
    cameraIntrinsics.at<double>(1, 2) = msg->K[5]; //cy
    cameraIntrinsics.at<double>(2, 2) = 1;
  }

  // Create the window to show HSR camera images
  cv::namedWindow(rgbName, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(depthName, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(planeName, cv::WINDOW_AUTOSIZE);

  // Set mouse handler for the window
  cv::setMouseCallback(rgbName, onMouseRGB);
  cv::setMouseCallback(depthName, onMouseDepth);

  ROS_INFO_STREAM("Subscribing to " << headStatusTopic << " ...");
  ros::Subscriber subhead = nh.subscribe(headStatusTopic, 1, headStatusCallback);

  // Define ROS topic from where HSR publishes images
  image_transport::ImageTransport it(nh2);
  // use compressed image transport to use less network bandwidth
  image_transport::TransportHints transportHint("compressed");

  ROS_INFO_STREAM("Subscribing to " << imageTopic << " ...");
  image_transport::Subscriber subrgb = it.subscribe(imageTopic, 1, imageCallback, transportHint);

  ROS_INFO_STREAM("Subscribing to " << depthTopic << " ...");
  ros::Subscriber subdepth = nh3.subscribe(depthTopic, 1, depthCallback);

  //ROS_INFO_STREAM("Subscribing to " << cloudTopic << " ...");
  //ros::Subscriber subcloud = nh4.subscribe(cloudTopic, 1, cloudCallback);

  ROS_INFO_STREAM("Subscribing to " << jointStatesTopic << " ...");
  ros::Subscriber subjoint = nh5.subscribe(jointStatesTopic, 1, jointStatesCallback);

  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::spin();

  cv::destroyWindow(rgbName);
  cv::destroyWindow(depthName);
  cv::destroyWindow(planeName);

  return EXIT_SUCCESS;
}
