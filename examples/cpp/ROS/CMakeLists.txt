#catkin_lint: ignore critical_var_overwrite
cmake_minimum_required(VERSION 3.0.2)
project(seerep_ros_examples VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC -DBOOST_LOG_DYN_LINK")

find_package(seerep_hdf5_ros REQUIRED)

find_package(SeerepHdf5Fb REQUIRED)
find_package(SeerepHdf5Core REQUIRED)
find_package(SeerepCoreFb REQUIRED)
find_package(SeerepCore REQUIRED)
find_package(seerep_ros_conversions_fb REQUIRED)
find_package(gRPC CONFIG REQUIRED)

find_package(
  catkin REQUIRED
  COMPONENTS std_msgs
             geometry_msgs
             sensor_msgs
             roscpp
             rosbag
             cv_bridge
             tf
)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(
  Boost
  COMPONENTS log log_setup
  REQUIRED
)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  sensor_msgs
  roscpp
  seerep_hdf5_ros
  seerep_ros_conversions_fb
  rosbag
  tf
)

include_directories(
  include
  ${SeerepHdf5Fb_INCLUDE_DIRS}
  ${SeerepHdf5Core_INCLUDE_DIRS}
  ${SeerepCoreFb_INCLUDE_DIRS}
  ${seerep_ros_conversions_fb_INCLUDE_DIRS}
  ${seerep_hdf5_ros_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_hdf5_node src/hdf5_node.cpp)

add_executable(${PROJECT_NAME}_rosbag_dumper src/rosbag_dumper.cpp)

add_dependencies(
  ${PROJECT_NAME}_hdf5_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  ${PROJECT_NAME}_rosbag_dumper ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}_hdf5_node ${seerep_hdf5_ros_LIBRARIES} ${catkin_LIBRARIES}
)

target_link_libraries(
  ${PROJECT_NAME}_rosbag_dumper
  ${SeerepHdf5Core_LIBRARIES}
  ${SeerepHdf5Fb_LIBRARIES}
  ${SeerepCoreFb_LIBRARIES}
  ${SeerepCore_LIBRARIES}
  ${seerep_ros_conversions_fb_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Boost_LOG_LIBRARY}
  ${Boost_LOG_SETUP_LIBRARY}
  gRPC::grpc++_reflection
  ${catkin_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}_hdf5_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
)
