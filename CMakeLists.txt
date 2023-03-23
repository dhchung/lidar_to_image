cmake_minimum_required(VERSION 3.0.2)
project(lidar_to_image)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
 CATKIN_DEPENDS cv_bridge image_transport pcl_conversions pcl_ros roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

