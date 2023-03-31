#catkin_lint: ignore critical_var_overwrite
#catkin_lint: ignore duplicate_find
cmake_minimum_required(VERSION 3.10)
project(seerep_hdf5_ros VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC")

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs sensor_msgs)

find_package(SeerepHdf5Core REQUIRED)

include(cmake/FindHighFive.cmake)
find_package(HighFive REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
add_definitions(-DBOOST_LOG_DYN_LINK)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  seerep_hdf5_ros
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  sensor_msgs
  DEPENDS
  HighFive
  SeerepHdf5Core
)

include_directories(
  include
  ${HighFive_INCLUDE_DIRS}
  ${SeerepHdf5Core_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/hdf5_ros.cpp)

target_link_libraries(
  ${PROJECT_NAME}
  ${HighFive_LIBRARIES}
  ${Boost_LIBRARIES}
  ${SeerepHdf5Core_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/seerep_hdf5_ros
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
)
