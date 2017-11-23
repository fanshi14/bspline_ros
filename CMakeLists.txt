cmake_minimum_required(VERSION 2.8)
project(bspline_ros)

set(PACKAGE_DEPENDENCIES
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)
add_message_files(FILES ControlPoints.msg)
generate_messages(DEPENDENCIES std_msgs)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include(cmake/DownloadTinyspline.cmake)
tinyspline_build(TINYSPLINE_LIBS TINYSPLINE_INCLUDE_DIR)
include_directories(include ${catkin_INCLUDE_DIRS} ${TINYSPLINE_INCLUDE_DIR})

add_definitions("${TINYSPLINE_DEFINITIONS}")

catkin_package(
  INCLUDE_DIRS include ${TINYSPLINE_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME} ${TINYSPLINE_LIBS}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

add_library(${PROJECT_NAME} src/bsplineGenerate.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${TINYSPLINE_LIBS})

add_executable(tinyspline_ros_node src/tinyspline_ros_node.cpp)
target_link_libraries(tinyspline_ros_node ${PROJECT_NAME}   ${catkin_LIBRARIES} ${TINYSPLINE_LIBS})
