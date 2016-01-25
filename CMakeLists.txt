cmake_minimum_required(VERSION 2.8.3)
project(dynamic_transform_publisher)

find_package(catkin REQUIRED COMPONENTS roscpp tf tf2_ros dynamic_reconfigure)

include(FindPkgConfig)

generate_dynamic_reconfigure_options(
  cfg/TF.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp tf tf2_ros dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynamic_transform_publisher src/dynamic_transform_publisher.cpp)

target_link_libraries(dynamic_transform_publisher
  ${catkin_LIBRARIES}
)
add_dependencies(dynamic_transform_publisher ${PROJECT_NAME}_gencfg)
add_dependencies(dynamic_transform_publisher ${catkin_EXPORTED_TARGETS})

install(TARGETS dynamic_transform_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
