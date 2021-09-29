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
  tf
  nav_core
  pluginlib
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
  LIBRARIES dubins dubins_planner
  CATKIN_DEPENDS project11 roscpp tf2 tf2_ros nav_core pluginlib
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

add_library(dubins_planner src/dubins_planner.cpp)
add_dependencies(dubins_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dubins_planner
    dubins
    ${catkin_LIBRARIES}
    )

install(TARGETS dubins_planner dubins
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
PATTERN ".svn" EXCLUDE
)

install(FILES dubins_planner_plugin.xml
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
