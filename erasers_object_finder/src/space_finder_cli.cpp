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
#include <erasers_nav_msgs/GetSpaceCentroid.h>

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
  erasers_nav_msgs::GetSpaceCentroid srv;
  ///////////

  double depth_min = 0.71;
  double depth_max = 0.94;
  double width_min = -0.25;
  double width_max = 0.25;
  double height_min = 0.85;
  double height_max = 1.02;
  double side_width = 0.10;
  double side_depth = 0.10;

  // Init the ROS node
  ros::init(argc, argv, "space_finder_cli");

  ROS_INFO("Starting space_finder_cli application ...");
 
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
	  if ( strcmp( argv[i], "-dmin") == 0 )
		  depth_min = atof(argv[++i]);

	  if ( strcmp( argv[i], "-dmax") == 0 )
		  depth_max = atof(argv[++i]);

	  if ( strcmp( argv[i], "-wmin") == 0 )
		  width_min = atof(argv[++i]);

	  if ( strcmp( argv[i], "-wmax") == 0 )
		  width_max = atof(argv[++i]);

	  if ( strcmp( argv[i], "-hmin") == 0 )
		  height_min = atof(argv[++i]);

	  if ( strcmp( argv[i], "-hmax") == 0 )
		  height_max = atof(argv[++i]);

	  if ( strcmp( argv[i], "-wside") == 0 )
		  side_width = atof(argv[++i]);

	  if ( strcmp( argv[i], "-dside") == 0 )
		  side_depth = atof(argv[++i]);

	  if ( strcmp( argv[i], "-debug") == 0 )
		  debug = true;
  }

  ROS_INFO_STREAM("Subscribing to /erasers/navigation/space_finder_srv ...");
  ros::ServiceClient client = nh.serviceClient<erasers_nav_msgs::GetSpaceCentroid>("/erasers/navigation/space_finder_srv");
  
  ros::Rate loop(10);
  while(ros::ok() && cv::waitKey(15) != 27)
  {
	  srv.request.depth_min = depth_min;
	  srv.request.depth_max = depth_max;
	  srv.request.width_min = width_min;
	  srv.request.width_max = width_max;
	  srv.request.height_min = height_min;
	  srv.request.height_max = height_max;
	  srv.request.side_width = side_width;
	  srv.request.side_depth = side_depth;

	  if (client.call(srv))
	  {
		  if (srv.response.space.id < 0)
			return 1;

		  if (!srv.response.space.isspace)
			return 1;

		  if (debug)
		  {
			cv::Mat p_fl, p_fr, p_bl, p_br;

			Transformations::VectMsg_ToCvMatf(srv.response.space.frontleft, 3, 1, p_fl);
			Transformations::VectMsg_ToCvMatf(srv.response.space.frontright, 3, 1, p_fr);
			Transformations::VectMsg_ToCvMatf(srv.response.space.backleft, 3, 1, p_bl);
			Transformations::VectMsg_ToCvMatf(srv.response.space.backright, 3, 1, p_br);

			std::cout << "space_finder_srv.frontleft: " << p_fl << std::endl;
			std::cout << "space_finder_srv.frontright: " << p_fr << std::endl;
			std::cout << "space_finder_srv.backleft: " << p_bl << std::endl;
			std::cout << "space_finder_srv.backright: " << p_br << std::endl;
		  }
	  } else
	  {
		  ROS_ERROR("Failed to call service: space_finder_cli");
		  return 1;
	  }

	  ros::spinOnce();
      loop.sleep();
  }

  return EXIT_SUCCESS;
}
