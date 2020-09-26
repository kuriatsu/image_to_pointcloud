cmake_minimum_required(VERSION 2.8.3)
project(image_to_pointcloud)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  pcl_ros
  roscpp
  rospy
)

find_package(OpenCV REQUIRED)
generate_dynamic_reconfigure_options(
   cfg/image_to_pointcloud.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_to_pointcloud
#  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs pcl_ros roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/image_to_pointcloud.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
