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
#include <erasers_nav_msgs/GetObjectsCentroid.h>

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
  erasers_nav_msgs::GetObjectsCentroid srv;
  ///////////

  double depth_min = 0.71;
  double depth_max = 0.94;
  double width_min = -0.25;
  double width_max = 0.25;
  double height_min = 0.85;
  double height_max = 1.02;
  double min_area = 250.0;
  double max_area = 100000.0;

  bool plane = false;
  bool bigplane = false;
  bool vertical = false;

  // Init the ROS node
  ros::init(argc, argv, "object_finder_cli");

  ROS_INFO("Starting object_finder_cli application ...");
 
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

	  if ( strcmp( argv[i], "-minarea") == 0 )
		  min_area = atof(argv[++i]);

	  if ( strcmp( argv[i], "-maxarea") == 0 )
		  max_area = atof(argv[++i]);

	  if ( strcmp( argv[i], "-debug") == 0 )
		  debug = true;

	  if ( strcmp( argv[i], "-plane") == 0 )
		  plane = true;

	  if ( strcmp( argv[i], "-bigplane") == 0 )
		  bigplane = true;

	  if ( strcmp( argv[i], "-vertical") == 0 )
		  vertical = true;
  }

  ROS_INFO_STREAM("Subscribing to /erasers/navigation/object_finder_srv ...");
  ros::ServiceClient client = nh.serviceClient<erasers_nav_msgs::GetObjectsCentroid>("/erasers/navigation/object_finder_srv");
  
  ros::Rate loop(10);
  while(ros::ok() && cv::waitKey(15) != 27)
  {
	  srv.request.depth_min = depth_min;
	  srv.request.depth_max = depth_max;
	  srv.request.width_min = width_min;
	  srv.request.width_max = width_max;
	  srv.request.height_min = height_min;
	  srv.request.height_max = height_max;
	  srv.request.min_area = min_area;
	  srv.request.max_area = max_area;
	  srv.request.plane = plane;
	  srv.request.bigplane = bigplane;
	  srv.request.vertical = vertical;

	  if (client.call(srv))
	  {
		  if (srv.response.objects.id < 0)
			return 1;

		  if (!srv.response.objects.isobject)
			return 1;

		  cv::Mat bbox, centroids;

		  Transformations::VectMsg_ToCvMatf(srv.response.objects.bbox, srv.response.objects.bb_w, srv.response.objects.bb_h, bbox);
		  Transformations::VectMsg_ToCvMatf(srv.response.objects.centroid, srv.response.objects.cntr_w, srv.response.objects.cntr_h, centroids);

		  if (debug)
		  {
			  for (int i = 0; i < srv.response.objects.n; i++)
			  {
				std::cout << "bbox " << i << ": " << bbox.row(i) << std::endl;
				std::cout << "centroid " << i << ": " << centroids.row(i) << std::endl;
			  }
		  }
	  } else
	  {
		  ROS_ERROR("Failed to call service: object_finder_cli");
		  return 1;
	  }

	  ros::spinOnce();
      loop.sleep();
  }

  return EXIT_SUCCESS;
}
