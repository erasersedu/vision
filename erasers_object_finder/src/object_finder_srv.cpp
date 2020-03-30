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
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>

// Erasers headers
#include <erasers_nav_msgs/GetObjectsCentroid.h>
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
static const std::string objectName       	= "Object Segmentation";
static const std::string planeName       	= "Plane Extraction";
static const std::string cameraFrame     	= "/xtion_rgb_optical_frame";
static const std::string imageTopic      	= "/hsrb/head_rgbd_sensor/rgb/image_rect_color";//"/xtion/rgb/image_raw";
static const std::string depthTopic      	= "/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw";//"/xtion/depth_registered/image_raw";
static const std::string cloudTopic      	= "/hsrb/head_rgbd_sensor/depth_registered/rectified_points"; //"/xtion/depth_registered/points";
static const std::string headStatusTopic  	= "/hsrb/head_trajectory_controller/state";
static const std::string jointStatesTopic 	= "/hsrb/joint_states";

//tf::TransformListener * transformListener;

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

  //transformListener = new tf::TransformListener();

  //tf::StampedTransform transform;
  //transformListener->waitForTransform("base_link", "head_rgbd_sensor_link", ros::Time(0), ros::Duration(10.0));
  //transformListener->lookupTransform("base_link", "head_rgbd_sensor_link", ros::Time(0), transform);

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

			/*int delta = 1;
			if ((h >= pointCloud.rows/2 - delta)&&(h <= pointCloud.rows/2 + delta)&&(w >= pointCloud.cols/2 - delta)&&(w <= pointCloud.cols/2 + delta))
			{
				pcl::PointXYZRGB pt = pclCloud->at(w, h);
				tf::Vector3 cubePosWrtRobot(pt.x, pt.y, pt.z);
				tf::Vector3 cubePosWrtWorld = transform * cubePosWrtRobot;

				std::cout << "self: " << p.x << ", " << p.y << ", " << p.z << std::endl;
				std::cout << "tf: " << cubePosWrtWorld.y() << ", " << cubePosWrtWorld.z() << ", " << cubePosWrtWorld.x() << std::endl;
			}*/
		}
	}

  return pointCloud;
}

//Orientation transformation stuff
cv::Mat changeOrientationTF(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud)
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
bool srvCallback(erasers_nav_msgs::GetObjectsCentroid::Request &req, erasers_nav_msgs::GetObjectsCentroid::Response &res)
{
  res.objects.id = idcount++;
  res.objects.isobject = false;
  res.objects.n = 0;

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

  cv::Mat plane_point = cv::Mat::zeros(1, 3, CV_32FC1);
  //Plane segmentation
  if (req.plane)
  {
	  if (req.vertical)
	  {
		  //Normal extraction
		  int blurSize = 5;

		  cv::Mat pointCloudBlur;
		  cv::blur(pointCloud, pointCloudBlur, cv::Size(blurSize, blurSize));
		  cv::Mat normals = getNormals(pointCloud, 2);

		  //Horizontal plane extraction
		  double yThreshold = 0.2;
		  cv::Mat  vNormals;
		  cv::inRange( normals, cv::Scalar(-1.0, 0.0, -1.0), cv::Scalar(1.0, yThreshold, 1.0), vNormals);

		  double delta_bb = 0.10;
		  cv::Mat validPointCloud;
		  cv::inRange(pointCloud, cv::Scalar(width_1, height_1, depth_1), 
			      cv::Scalar(width_2, height_2, depth_2), validPointCloud);
		  vNormals = vNormals&validPointCloud;

		  //cv::imshow("vNormals", vNormals);

		  //Plane extraction with RANSAC
		  double maxDistPointToPlane = 0.01;
		  int maxIterations = 100;
		  int minPointsForPlane = pointCloud.rows*pointCloud.cols*0.01;
		  plane = getVerticalPlane(pointCloud, vNormals, maxDistPointToPlane, maxIterations, minPointsForPlane);

		  if (!plane.isplane)
		  {
			std::cout << "object_finder_srv.empty" << std::endl;
			return true;
		  }

		  plane_point.at<float>(0) = (float)plane.point.x;
		  plane_point.at<float>(1) = (float)plane.point.y;
		  plane_point.at<float>(2) = (float)plane.point.z;

		  depth_2 = plane.point.z;// - 0.01;
		  //cv::imshow("hNormalsRANSAC", plane.bigplane);
	  } else
	  {
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
		  double maxDistPointToPlane = 0.01;
		  int maxIterations = 100;
		  int minPointsForPlane = pointCloud.rows*pointCloud.cols*0.01;
		  plane = getHorizontalPlane(pointCloud, hNormals, maxDistPointToPlane, maxIterations, minPointsForPlane);

		  if (!plane.isplane)
		  {
			std::cout << "object_finder_srv.empty" << std::endl;
			return true;
		  }

		  plane_point.at<float>(0) = (float)plane.point.x;
		  plane_point.at<float>(1) = (float)plane.point.y;
		  plane_point.at<float>(2) = (float)plane.point.z;

		  height_2 = height_2 - height_1;
		  height_1 = plane.point.y + 0.01;
		  height_2 = height_1 + height_2;
		  //cv::imshow("hNormalsRANSAC", plane.bigplane);
	  }
  }

  //////////
  //Spatial segmentation
  int delta_w = 20, delta_h = 25;
  double minDistToContour = 0.01;

  for (int h = 0; h < pointCloud.rows - delta_h; h++)
  	for (int w = 0 + delta_w; w < pointCloud.cols - 2*delta_w; w++)
	{
		cv::Point3f p = pointCloud.at<cv::Vec3f>(h, w);
		if (req.plane)
		{
			//Plane masking
			if (req.bigplane)
			{
				if (req.vertical)
				{
					if ( !std::isnan(pclCloud->at(w, h).z) && 
					     cv::pointPolygonTest( plane.convexHullXY, cv::Point2f( p.x, p.y ), true) > minDistToContour && 
					     !plane.bigplane.data[h*plane.bigplane.step + w] )
					{
						if (p.x > width_1 && p.x < width_2 &&
						    p.y > height_1 && p.y < height_2 &&
						    p.z > depth_1 && p.z < depth_2)
						{
							//3D clustering
							hw_point.at<float>(0) = (float)h;
							hw_point.at<float>(1) = (float)w;

							xyz_point.at<float>(0) = (float)p.x;
							xyz_point.at<float>(1) = (float)p.y;
							xyz_point.at<float>(2) = (float)p.z;

							hw_samples.push_back(hw_point);
							xyz_samples.push_back(xyz_point);
						}
					}
				} else
				{
					if ( !std::isnan(pclCloud->at(w, h).z) && 
					     cv::pointPolygonTest( plane.convexHullXZ, cv::Point2f( p.x, p.z ), true) > minDistToContour && 
					     !plane.bigplane.data[h*plane.bigplane.step + w] )
					{
						if (p.x > width_1 && p.x < width_2 &&
						    p.y > height_1 && p.y < height_2 &&
						    p.z > depth_1 && p.z < depth_2)
						{
							//3D clustering
							hw_point.at<float>(0) = (float)h;
							hw_point.at<float>(1) = (float)w;

							xyz_point.at<float>(0) = (float)p.x;
							xyz_point.at<float>(1) = (float)p.y;
							xyz_point.at<float>(2) = (float)p.z;

							hw_samples.push_back(hw_point);
							xyz_samples.push_back(xyz_point);
						}
					}
				}
			} else
			{
				if (req.vertical)
				{
					if ( !std::isnan(pclCloud->at(w, h).z) && 
					     cv::pointPolygonTest( plane.convexHullXY, cv::Point2f( p.x, p.y ), true) > minDistToContour && 
					     !plane.plane.data[h*plane.plane.step + w] )
					{
						if (p.x > width_1 && p.x < width_2 &&
						    p.y > height_1 && p.y < height_2 &&
						    p.z > depth_1 && p.z < depth_2)
						{
							//3D clustering
							hw_point.at<float>(0) = (float)h;
							hw_point.at<float>(1) = (float)w;

							xyz_point.at<float>(0) = (float)p.x;
							xyz_point.at<float>(1) = (float)p.y;
							xyz_point.at<float>(2) = (float)p.z;

							hw_samples.push_back(hw_point);
							xyz_samples.push_back(xyz_point);
						}
					}
				} else
				{
					if ( !std::isnan(pclCloud->at(w, h).z) && 
					     cv::pointPolygonTest( plane.convexHullXZ, cv::Point2f( p.x, p.z ), true) > minDistToContour && 
					     !plane.plane.data[h*plane.plane.step + w] )
					{
						if (p.x > width_1 && p.x < width_2 &&
						    p.y > height_1 && p.y < height_2 &&
						    p.z > depth_1 && p.z < depth_2)
						{
							//3D clustering
							hw_point.at<float>(0) = (float)h;
							hw_point.at<float>(1) = (float)w;

							xyz_point.at<float>(0) = (float)p.x;
							xyz_point.at<float>(1) = (float)p.y;
							xyz_point.at<float>(2) = (float)p.z;

							hw_samples.push_back(hw_point);
							xyz_samples.push_back(xyz_point);
						}
					}
				}
			}
		} else
		{
			//Bounding box masking
			if ( !std::isnan(pclCloud->at(w, h).z) )
			{
				if (p.x > width_1 && p.x < width_2 &&
				    p.y > height_1 && p.y < height_2 &&
				    p.z > depth_1 && p.z < depth_2)
				{
					//3D clustering
					hw_point.at<float>(0) = (float)h;
					hw_point.at<float>(1) = (float)w;

					xyz_point.at<float>(0) = (float)p.x;
					xyz_point.at<float>(1) = (float)p.y;
					xyz_point.at<float>(2) = (float)p.z;

					hw_samples.push_back(hw_point);
					xyz_samples.push_back(xyz_point);
				}
			}
		}
	}

  if (xyz_samples.rows < 100)
  {
	std::cout << "object_finder_srv.empty" << std::endl;
	return true;
  }

  //////////
  //3D (xyz) clustering
  int n_clusters = 250;
  double min_distance = 0.02;

  cv::Mat centers;
  cv::Mat clusters = cv::Mat::zeros(xyz_samples.rows, 1, CV_32SC1);
  cv::Mat labels = cv::Mat::zeros(n_clusters, 1, CV_32SC1) - 1;

  try {
    cv::kmeans(xyz_samples, n_clusters, clusters, cvTermCriteria(CV_TERMCRIT_EPS|CV_TERMCRIT_ITER, 10, 1.0),
	     1, cv::KMEANS_PP_CENTERS, centers);
  } catch (const std::exception &e)
  {
  	std::cout << "object_finder_srv.kmeans_empty" << std::endl;
	return true;
  }
  //Centroid grouping by distance
  int lbl = 0;
  for (int i = 0; i < n_clusters; i++)
      for (int j = 0; j < n_clusters; j++)
	  {
		cv::Mat a = centers.row(i);
		cv::Mat b = centers.row(j);

		//Get distance on the horizontal plane
		double dist = sqrt( (a.at<float>(0)-b.at<float>(0))*(a.at<float>(0)-b.at<float>(0)) + 
				    (a.at<float>(2)-b.at<float>(2))*(a.at<float>(2)-b.at<float>(2)) );

		if (dist < min_distance)
		{
			if (labels.at<int>(j) == -1)
			{
				if (labels.at<int>(i) == -1)
				{
				   labels.at<int>(j) = lbl++;
				   labels.at<int>(i) = labels.at<int>(j);
				}
				else
				{
				   labels.at<int>(j) = labels.at<int>(i);
				}
			}
			else
			{
				if (labels.at<int>(i) == -1)
				{
					labels.at<int>(i) = labels.at<int>(j);
				}
				else
				{
					if (labels.at<int>(i) < labels.at<int>(j))
					{
					   for (int x = 0; x < labels.rows; x++)
						{
						  if (labels.at<int>(x) == labels.at<int>(j))
					   	     labels.at<int>(x) = labels.at<int>(i);
						}
					}
					else
					{
					   for (int x = 0; x < labels.rows; x++)
						{
						  if (labels.at<int>(x) == labels.at<int>(i))
					   	     labels.at<int>(x) = labels.at<int>(j);
						}
					}  
 				}
			}
		}
	  }

  //Object counting
  std::vector<int> lbl_valid;
  lbl_valid.push_back( labels.at<int>(0) );
  for (int i = 1; i < labels.rows; i++) 
      {
	bool lbl_new = true;
	for (int j = 0; j < lbl_valid.size(); j++)
	    {
		if (labels.at<int>(i) == lbl_valid[j])
		   lbl_new = false;
	    }

	if (lbl_new)
	   lbl_valid.push_back( labels.at<int>(i) );
      }

  //Random colours generation
  std::vector<cv::Scalar> colors;
  for (int i = 0; i < lbl_valid.size(); i++) 
      {
	cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
	colors.push_back(color);
      }

  //Object segmentation by size
  cv::Mat bbmat, eigenvectors, eigenvalues, eigencenters;
  cv::Mat bgr = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_8UC3);

  for (int i = 0; i < lbl_valid.size(); i++)
      {
	  //Single object mask
	  cv::Mat mask = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_8UC1);
	  cv::Scalar color = colors[i];

	  for (int j=0; j < clusters.rows; j++)
	      {
		int idx = clusters.at<int>(j);
		if (labels.at<int>(idx) == lbl_valid[i])
		{
			int h = (int)hw_samples.at<float>(j,0);
			int w = (int)hw_samples.at<float>(j,1);

			pcl::PointXYZRGB p = pclCloud->at(w, h);

			bgr.data[h*bgr.step + w*3] = (unsigned char)((p.b+color.val[0])/2);
			bgr.data[h*bgr.step + w*3 + 1] = (unsigned char)((p.g+color.val[1])/2);
			bgr.data[h*bgr.step + w*3 + 2] = (unsigned char)((p.r+color.val[2])/2);

			mask.data[h*mask.step + w] = (unsigned char)255;
		}
	      }

	  //Bounding boxing
	  std::vector<std::vector<cv::Point> > contours;
	  std::vector<cv::Vec4i> hierarchy;

	  cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	  std::vector<cv::Rect> brects;
	  for (int idx = 0 ; idx < (int)contours.size(); idx++ )
	  {
		  double area = cv::contourArea(contours[idx], false);
		  if ((area > req.min_area)&&(area < req.max_area))
		  {
			cv::Rect brect = cv::boundingRect (contours[idx]);
			cv::rectangle(bgr, brect, color, 3, 4);

			brects.push_back(brect);
		  }
	  }

	  //PCA calculation
	  if (brects.size() > 0)
	  {
		  cv::Mat xyz_pca; 
		  for (int i = 0; i < brects.size(); i++)
		  {
		  	int w_1 = brects[i].x, h_1 = brects[i].y;
		  	int w_2 = brects[i].x + brects[i].width;
			int h_2 = brects[i].y + brects[i].height;

			cv::Mat pca_point = cv::Mat::zeros(1, 3, CV_64FC1);

		  	for (int h = h_1; h < h_2; h++)
		  	  for (int w = w_1; w < w_2; w++)
			  {
				if (mask.data[h*mask.step + w] != 0 && 
				    !std::isnan(pclCloud->at(w, h).z) )
				{
					pca_point.at<double>(0) = pointCloud.at<cv::Vec3f>(h,w)[0]*1000;
					pca_point.at<double>(1) = pointCloud.at<cv::Vec3f>(h,w)[1]*1000;
					pca_point.at<double>(2) = pointCloud.at<cv::Vec3f>(h,w)[2]*1000;

					xyz_pca.push_back(pca_point);

					//counter++;
				}		
			  }

			//Valid bounding box extraction
			cv::Mat bb = cv::Mat::zeros(1, 4, CV_32SC1);
			bb.at<float>(0) = brects[i].x;
			bb.at<float>(1) = brects[i].y;
			bb.at<float>(2) = brects[i].width;
			bb.at<float>(3) = brects[i].height;
			bbmat.push_back(bb);

			//PCA calculation
			cv::PCA pca(xyz_pca, cv::Mat(), CV_PCA_DATA_AS_ROW);

			cv::Mat eigencenter = cv::Mat::zeros(1, 3, CV_32FC1);
			eigencenter.at<float>(0) = (float)pca.mean.at<double>(0, 0)/1000;
			eigencenter.at<float>(1) = (float)pca.mean.at<double>(0, 1)/1000;
			eigencenter.at<float>(2) = (float)pca.mean.at<double>(0, 2)/1000;
			eigencenters.push_back(eigencenter);

			cv::Mat eigenvector = cv::Mat::zeros(1, 3, CV_32FC1);
			cv::Mat eigenvalue = cv::Mat::zeros(1, 3, CV_32FC1);
			for (int id = 0; id < 3; id++)
			    {
				eigenvector.at<float>(0) = (float)pca.eigenvectors.at<double>(id, 0);
				eigenvector.at<float>(1) = (float)pca.eigenvectors.at<double>(id, 1);
				eigenvector.at<float>(2) = (float)pca.eigenvectors.at<double>(id, 2);

				eigenvectors.push_back(eigenvector);
				eigenvalue.at<float>(id) = (float)pca.eigenvalues.at<double>(id);
			    }

			eigenvalues.push_back( eigenvalue );
		  }
	  }
      }

  //Eigenvectors drawing
  if (bbmat.rows > 0)
  {
	for (int i = 0; i < bbmat.rows; i++)
	    {
		int x_center = (int)(bbmat.at<float>(i,0) + bbmat.at<float>(i,2)/2);
		int y_center = (int)(bbmat.at<float>(i,1) + bbmat.at<float>(i,3)/2);
		
		int x_length = 25, y_length = 25;
		bool front_grasping = true;
		if ( fabs(eigenvectors.at<float>(3*i,0)) > fabs(eigenvectors.at<float>(3*i,1)) || 
		     fabs(eigenvectors.at<float>(3*i,2)) > fabs(eigenvectors.at<float>(3*i,1)) )
		    front_grasping = false;

		//if (!front_grasping && 
		//    ( fabs( eigenvalues.at<float>(i, 0)/eigenvalues.at<float>(i, 1) ) < 4 ||
		//      fabs( eigenvalues.at<float>(i, 0)/eigenvalues.at<float>(i, 2) ) < 4 ) )
		//    front_grasping = true;

		if(front_grasping)
		   y_length = 50;
		else
		   x_length = 50;

		cv::Point o, p, q, a;
		o.x = x_center; o.y = y_center;
		p.x = o.x + x_length; p.y = o.y;
		if (p.x >= bgr.cols)
		   p.x = bgr.cols - 1;

		q.x = o.x; q.y = o.y - y_length;
		if (q.y < 0)
		   q.y = 0;

		cv::circle(bgr, o, 3, cv::Scalar(255, 0, 255), 2);

		cv::line(bgr, o, p, cv::Scalar(0, 0, 255), 2, CV_AA);
		a.x = p.x - 5; a.y = p.y - 5;
		cv::line(bgr, p, a, cv::Scalar(0, 0, 255), 2, CV_AA);
		a.x = p.x - 5; a.y = p.y + 5;
		cv::line(bgr, p, a, cv::Scalar(0, 0, 255), 2, CV_AA);

		cv::line(bgr, o, q, cv::Scalar(0, 255, 0), 2, CV_AA);
		a.x = q.x - 5; a.y = q.y + 5;
		cv::line(bgr, q, a, cv::Scalar(0, 255, 0), 2, CV_AA);
		a.x = q.x + 5; a.y = q.y + 5;
		cv::line(bgr, q, a, cv::Scalar(0, 255, 0), 2, CV_AA);
	    }
  }

  //Response message generation
  int vec_w, vec_h;
  std::vector<float_t> vec_bb, vec_centroids, vec_evec, vec_eval, vec_ppoint;

  if (eigencenters.rows > 0)
  {
	  res.objects.isobject = true;

	  res.objects.n = eigencenters.rows;

	  Transformations::CvMatf_ToVectMsg(bbmat, &vec_bb, vec_w, vec_h);
	  res.objects.bb_w = vec_w;
	  res.objects.bb_h = vec_h;
	  res.objects.bbox = vec_bb;

	  Transformations::CvMatf_ToVectMsg(eigencenters, &vec_centroids, vec_w, vec_h);
	  res.objects.cntr_w = vec_w;
	  res.objects.cntr_h = vec_h;
	  res.objects.centroid = vec_centroids;

	  Transformations::CvMatf_ToVectMsg(eigenvectors, &vec_evec, vec_w, vec_h);
	  res.objects.evec_w = vec_w;
	  res.objects.evec_h = vec_h;
	  res.objects.eigenvectors = vec_evec;

	  Transformations::CvMatf_ToVectMsg(eigenvalues, &vec_eval, vec_w, vec_h);
	  res.objects.eval_w = vec_w;
	  res.objects.eval_h = vec_h;
	  res.objects.eigenvalues = vec_eval;

	  Transformations::CvMatf_ToVectMsg(plane_point, &vec_ppoint, vec_w, vec_h);
	  res.objects.plane_point = vec_ppoint;

	  pcl::toROSMsg(*pclCloud, res.objects.pcl);
  }

  if (debug)
  {
	  if (eigencenters.rows > 0)
	  {
		  ROS_INFO_STREAM("Pitch angle: " << (-1)*angle_pitch * 180 / 3.14159265358979323846);

		  for (int i = 0; i < eigencenters.rows; i++)
		  {
			std::cout << "object_finder_srv.eigencenters[" << i << "]: " << eigencenters.row(i) << std::endl;
		  }

		  if (req.plane)
		  	std::cout << "object_finder_srv.plane: " << plane.point << std::endl;

		  //for (int i = 0; i < eigenvalues.rows; i++)
		  //{
		  //	std::cout << "object_finder_srv.eigenvalues[" << i << "]: " << eigenvalues.row(i) << std::endl;
		  //}

		  //for (int i = 0; i < eigenvectors.rows; i++)
		  //{
		  //	std::cout << "object_finder_srv.eigenvectors[" << i << "]: " << eigenvectors.row(i) << std::endl;
		  //}

		  cv::imshow(objectName, bgr);
		  if (req.bigplane)
		  	cv::imshow(planeName, plane.bigplane);
		  else
			cv::imshow(planeName, plane.plane);

		  cv::waitKey(15);
	  }
	  else
	  {
		std::cout << "object_finder_srv.empty" << std::endl;
	  }
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "object_finder_srv");

  ROS_INFO("Starting object_finder_srv application ...");
 
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
	  cv::namedWindow(planeName, cv::WINDOW_AUTOSIZE);
	  cv::namedWindow(objectName, cv::WINDOW_AUTOSIZE);
  }

  ros::ServiceServer service = nh.advertiseService("/erasers/navigation/object_finder_srv", srvCallback);

  ROS_INFO_STREAM("Subscribing to " << headStatusTopic << " ...");
  ros::Subscriber subhead = nh2.subscribe(headStatusTopic, 1, headStatusCallback);

  // Define ROS topic from where HSR publishes images
  image_transport::ImageTransport it(nh3);
  // use compressed image transport to use less network bandwidth
  image_transport::TransportHints transportHint("compressed");

  //transformListener = new tf::TransformListener();

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
	  cv::destroyWindow(planeName);
	  cv::destroyWindow(objectName);
  }

  return EXIT_SUCCESS;
}
