cmake_minimum_required(VERSION 2.8.3)
project(scan2pcl)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  roscpp
  std_msgs
  tf
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(scan2pcl src/scan2pcl.cpp)
target_link_libraries(scan2pcl ${catkin_LIBRARIES})

