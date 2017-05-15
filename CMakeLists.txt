cmake_minimum_required(VERSION 2.8.3)
find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
project(scan2pcl)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  roscpp
  std_msgs
  tf
  pcl_ros
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_package(CATKIN_DEPENDS DEPENDS Eigen)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(scan2pcl src/scan2pcl.cpp)
target_link_libraries(scan2pcl ${catkin_LIBRARIES})

add_executable(pcl2global src/pcl2global.cpp)
target_link_libraries(pcl2global ${catkin_LIBRARIES})

add_executable(pcl2odomc src/pcl2odomc.cpp)
target_link_libraries(pcl2odomc ${catkin_LIBRARIES})

add_executable(map2odomtf src/map2odomtf.cpp)
target_link_libraries(map2odomtf ${catkin_LIBRARIES})