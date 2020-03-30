cmake_minimum_required(VERSION 2.8.3)
project(erasers_object_finder)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  
  actionlib

  pcl_conversions
  pcl_ros

  dynamic_reconfigure
  tf

  cv_bridge
  image_transport

  trajectory_msgs

  erasers_tools
  erasers_nav_msgs
  sensor_msgs
  control_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
set(CMAKE_PREFIX_PATH "/usr/local/")
set(OpenCV_INCLUDE_DIRS "/usr/local/include")

catkin_package(
 CATKIN_DEPENDS erasers_tools erasers_nav_msgs
)

include_directories(SYSTEM 
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(rgbd_src_node src/rgbd_src_node.cpp)
target_link_libraries(rgbd_src_node
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})

add_executable(object_finder_srv src/object_finder_srv.cpp)
target_link_libraries(object_finder_srv
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})

add_executable(object_finder_cli src/object_finder_cli.cpp)
target_link_libraries(object_finder_cli
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})

add_executable(space_finder_srv src/space_finder_srv.cpp)
target_link_libraries(space_finder_srv
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})

add_executable(space_finder_cli src/space_finder_cli.cpp)
target_link_libraries(space_finder_cli
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})

### INSTALL ###

install(TARGETS rgbd_src_node object_finder_srv object_finder_cli space_finder_srv space_finder_cli
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
