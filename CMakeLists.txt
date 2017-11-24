cmake_minimum_required(VERSION 2.8.3)
project(dynamic_transform_publisher)

find_package(catkin REQUIRED COMPONENTS roscpp tf2_ros dynamic_reconfigure interactive_markers visualization_msgs)

include(FindPkgConfig)

generate_dynamic_reconfigure_options(
  cfg/TF.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamic_tf2
  CATKIN_DEPENDS roscpp tf2_ros dynamic_reconfigure interactive_markers visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_library(dynamic_tf2
  src/dynamic_transform_broadcaster.cpp
  src/dynamic_transform_marker.cpp
)
target_link_libraries(dynamic_tf2 ${catkin_LIBRARIES})
add_dependencies(dynamic_tf2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dynamic_transform_publisher src/dynamic_transform_publisher.cpp)
target_link_libraries(dynamic_transform_publisher dynamic_tf2)

install(TARGETS dynamic_transform_publisher dynamic_tf2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
