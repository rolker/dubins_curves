cmake_minimum_required(VERSION 2.8.3)
project(dubins_curves)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
  geographic_msgs
  project11
)

add_service_files(
  FILES
  DubinsCurves.srv
  DubinsCurvesLatLong.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  geographic_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dubins
  CATKIN_DEPENDS project11 roscpp geometry_msgs geographic_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(dubins src/dubins.c)

add_executable(${PROJECT_NAME}_node src/dubins_curves_node.cpp src/dubins.c)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
