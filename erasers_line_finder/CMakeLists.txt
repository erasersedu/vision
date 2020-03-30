cmake_minimum_required(VERSION 2.8.3)
project(erasers_line_finder)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  
  actionlib
  
  erasers_tools

  erasers_nav_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  control_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
set(CMAKE_PREFIX_PATH "/usr/local/")
set(OpenCV_INCLUDE_DIRS "/usr/local/include")

catkin_package()

include_directories(SYSTEM 
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(front_line_srv src/front_line_srv.cpp )
target_link_libraries(front_line_srv 
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})

add_executable(front_line_cli src/front_line_cli.cpp )
target_link_libraries(front_line_cli 
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})

add_executable(front_point_srv src/front_point_srv.cpp )
target_link_libraries(front_point_srv 
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})

add_executable(front_point_cli src/front_point_cli.cpp )
target_link_libraries(front_point_cli 
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})

### INSTALL ###

#install(TARGETS ${PROJECT_NAME} rgbd_src top_pov
install(TARGETS front_line_srv front_line_cli front_point_srv front_point_cli
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
