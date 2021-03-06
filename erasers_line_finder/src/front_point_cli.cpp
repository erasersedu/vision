// C++ standard headers
#include <exception>
#include <string>
#include <math.h>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <ros/topic.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// PCL headers
#include <pcl/point_cloud.h>

// erasers headers
#include <erasers_nav_msgs/ProjectedView.h>
//#include <erasers_nav_msgs/GetProjectedView.h>
#include <erasers_nav_msgs/GetClosestPoint.h>

#include <erasers_tools/transformations.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string rgbName         = "Point_Client_RGB";
static const std::string povName         = "Point_Client_POV";

//Global parameters
bool debug = false;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  //Server parameters
  erasers_nav_msgs::GetClosestPoint srv;
  ///////////

  // Init the ROS node
  ros::init(argc, argv, "front_point_cli");

  ROS_INFO("Starting front_point_cli application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  for (int i = 0; i < argc; i++)
  {
	  //std::cout << argv[i] << std::endl;
	  if ( strcmp( argv[i], "-debug") == 0 )
		  debug = true;
  }

  // Create the window to show TIAGo's camera images
  /*if (debug)
  {
	cv::namedWindow(rgbName, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(povName, cv::WINDOW_AUTOSIZE);
  }*/

  ROS_INFO_STREAM("Subscribing to /erasers/navigation/front_point_srv ...");
  ros::ServiceClient client = nh.serviceClient<erasers_nav_msgs::GetClosestPoint>("/erasers/navigation/front_point_srv");
  
  ros::Rate loop(10);
  while(ros::ok() && cv::waitKey(15) != 27)
  {
	  srv.request.maxx = -1;
	  srv.request.miny = -1;
	  srv.request.maxy = -1;
	  srv.request.maxz = -1;
	  if (client.call(srv))
	  {
		  if (srv.response.closest_point.id < 0)
			return 1;

		  if (!srv.response.closest_point.ispoint)
			return 1;

		  if ( (srv.response.closest_point.point_w * srv.response.closest_point.point_h) == 0)
			return 1;

		  std_msgs::Header header = srv.response.closest_point.header;

		  cv::Mat metric_point, pixel_point;
		  int w, h;

		  w = srv.response.closest_point.point_w;
		  h = srv.response.closest_point.point_h;
		  Transformations::VectMsg_ToCvMatf(srv.response.closest_point.metric_point, w, h, metric_point);
		  Transformations::VectMsg_ToCvMatf(srv.response.closest_point.pixel_point, w, h, pixel_point);

		  if (debug)
		  {
			std::cout << "Stamp: " << header.stamp << ", Seq: " << header.seq << ", Frame ID: " << header.frame_id << std::endl;

			std::cout << "LineFinder.->X1: " << metric_point.at<float>(0) << ", Y1: " << metric_point.at<float>(1) << ", Z1: " << metric_point.at<float>(2) << std::endl;
			std::cout << "LineFinder.->P1X: " << (int)pixel_point.at<float>(0) << ", P1Y: " << (int)pixel_point.at<float>(1) << std::endl;
		  }
	  } else
	  {
		  ROS_ERROR("Failed to call service: front_point_srv");
		  return 1;
	  }
	  
	  ros::spinOnce();
      loop.sleep();
  }

  /*if (debug)
  {
	cv::destroyWindow(rgbName);
	cv::destroyWindow(povName);
  }*/

  return EXIT_SUCCESS;
}
