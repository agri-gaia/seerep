cmake_minimum_required(VERSION 3.0.2)
project(seerep-core-fb VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC -DBOOST_LOG_DYN_LINK")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(FindFlatBuffers)

find_package(SeerepHdf5Fb REQUIRED)
find_package(seerep_ros_conversions_fb REQUIRED)
find_package(Flatbuffers REQUIRED)
find_package(SeerepMsgs REQUIRED)
find_package(HighFive REQUIRED)
find_package(
  Eigen3
  3.3
  REQUIRED
  NO_MODULE
)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(
  Boost
  COMPONENTS log log_setup
  REQUIRED
)

find_package(catkin REQUIRED COMPONENTS tf2)

configure_file(include/SeerepCoreFbConfig.h.in SeerepCoreFbConfig.h)

include_directories(
  include
  ${SeerepMsgs_INCLUDE_DIRS}
  ${SeerepHdf5Fb_INCLUDE_DIRS}
  ${seerep_ros_conversions_fb_INCLUDE_DIRS}
  ${FLATBUFFER_INCLUDE_DIR}
  ${HighFive_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(
  seerepcorefb
  src/core-fb-conversion.cpp
  src/core-fb-image.cpp
  src/core-fb-instance.cpp
  src/core-fb-point.cpp
  src/core-fb-tf.cpp
  src/core-fb-pointcloud.cpp
  src/core-fb-camera-intrinsics.cpp
)

target_link_libraries(
  seerepcorefb
  ${SeerepMsgs_LIBRARIES}
  ${SeerepHdf5Fb_LIBRARIES}
  ${seerep_ros_conversions_fb_LIBRARIES}
  ${FLATBUFFER_LIBRARIES}
  ${HighFive_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Boost_LOG_LIBRARY}
  ${Boost_LOG_SETUP_LIBRARY}
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

install(
  TARGETS seerepcorefb
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
include(CMakePackageConfigHelpers)

configure_package_config_file(
  cmake/SeerepCoreFbConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/SeerepCoreFbConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SeerepCoreFbConfigVersion.cmake"
  VERSION "${seerep_fb_core_VERSION_MAJOR}.${seerep_fb_core_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

# install the configuration file
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/SeerepCoreFbConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/SeerepCoreFbConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/seerepcorefb
)

install(DIRECTORY include/seerep-core-fb
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES "${PROJECT_BINARY_DIR}/SeerepCoreFbConfig.h"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
