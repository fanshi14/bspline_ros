cmake_minimum_required(VERSION 3.4)#2.8.3)
project(bspline_ros)

set(PACKAGE_DEPENDENCIES
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)
set(CMAKE_CXX_FLAGS "-std=c++11")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_subdirectory(bspline)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/bspline/tinyspline/build/lib)
include_directories(include ${CMAKE_CURRENT_BINARY_DIR}/bspline/tinyspline/library)
#add_definitions(-DUSE_TINYSPLINE)
add_definitions("${TINYSPLINE_DEFINITIONS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS system_lib
)

###########
## Build ##
###########

set(LINK_LIBS
  ${TINYSPLINE_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME} src/bsplineGenerate.cpp)
target_link_libraries(${PROJECT_NAME}
  LINK_PUBLIC tinysplinecpp
  ${LINK_LIBS}
)


add_executable(tinyspline_ros_node src/tinyspline_ros_node.cpp)
target_link_libraries(tinyspline_ros_node
  LINK_PUBLIC tinysplinecpp
  ${PROJECT_NAME}
  ${LINK_LIBS}
  )
set_target_properties(tinyspline_ros_node PROPERTIES FOLDER "src")

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS tinyspline_ros tinyspline_ros_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tinyspline_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
