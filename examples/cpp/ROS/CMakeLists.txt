#catkin_lint: ignore critical_var_overwrite
cmake_minimum_required(VERSION 3.0.2)
project(seerep_ros_examples VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")

find_package(seerep_hdf5_ros REQUIRED)

find_package(
  catkin REQUIRED
  COMPONENTS std_msgs
             geometry_msgs
             sensor_msgs
             roscpp
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
)

include_directories(
  include ${seerep_hdf5_ros_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_hdf5_node src/hdf5_node.cpp)

add_dependencies(
  ${PROJECT_NAME}_hdf5_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}_hdf5_node ${seerep_hdf5_ros_LIBRARIES} ${catkin_LIBRARIES}
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
