cmake_minimum_required(VERSION 3.0.2)
project(seerep_server VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC -DBOOST_LOG_DYN_LINK")

# Add a custom command that produces version.cpp, plus a dummy output that's not
# actually produced, in order to force version.cmake to always be re-run before
# the build
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/version.cpp
         ${CMAKE_CURRENT_BINARY_DIR}/_version.cpp
  COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/version.cmake
)

find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(SeerepHdf5Pb REQUIRED)
find_package(SeerepHdf5Fb REQUIRED)
find_package(SeerepHdf5Core REQUIRED)
find_package(SeerepCom REQUIRED)
find_package(SeerepCore REQUIRED)
find_package(SeerepCorePb REQUIRED)
find_package(SeerepCoreFb REQUIRED)
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
  COMPONENTS program_options log log_setup
  REQUIRED
)

find_package(
  catkin REQUIRED COMPONENTS seerep_ros_conversions_pb
                             seerep_ros_conversions_fb
)

include_directories(
  include
  ${SeerepCom_INCLUDE_DIRS}
  ${SeerepCore_INCLUDE_DIRS}
  ${SeerepCorePb_INCLUDE_DIRS}
  ${SeerepCoreFb_INCLUDE_DIRS}
  ${SeerepHdf5Pb_INCLUDE_DIRS}
  ${SeerepHdf5Fb_INCLUDE_DIRS}
  ${SeerepHdf5Core_INCLUDE_DIRS}
  ${HighFive_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}
  src/server.cpp
  src/pb_meta_operations.cpp
  src/pb_tf_service.cpp
  src/pb_image_service.cpp
  src/pb_point_cloud_service.cpp
  src/pb_camera_intrinsics_service.cpp
  src/fb_meta_operations.cpp
  src/fb_point_service.cpp
  src/fb_tf_service.cpp
  src/fb_image_service.cpp
  src/fb_instance_service.cpp
  src/fb_point_cloud_service.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/version.cpp
  src/fb_cameraintrinsics_service.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${SeerepHdf5Pb_LIBRARIES}
  ${SeerepHdf5Fb_LIBRARIES}
  ${SeerepHdf5Core_LIBRARIES}
  ${SeerepCom_LIBRARIES}
  ${SeerepCore_LIBRARIES}
  ${SeerepCorePb_LIBRARIES}
  ${SeerepCoreFb_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Boost_LOG_LIBRARY}
  ${Boost_LOG_SETUP_LIBRARY}
  gRPC::grpc++_reflection
  Eigen3::Eigen
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
