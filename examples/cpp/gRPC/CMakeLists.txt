cmake_minimum_required(VERSION 3.0.2)
project(examples_cpp VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(SeerepCom REQUIRED)
find_package(SeerepMsgs REQUIRED)

find_package(Flatbuffers REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(
  Boost
  COMPONENTS program_options log log_setup
  REQUIRED
)

include_directories(
  include
  ${SeerepCom_INCLUDE_DIRS}
  ${SeerepMsgs_INCLUDE_DIRS}
  ${FLATBUFFER_INCLUDE_DIR}
  ${PROTOBUF_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_queryImages src/queryImages.cpp)

set(common_link_libs
    ${SeerepCom_LIBRARIES}
    ${seerep_ros_conversions_pb_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Boost_LOG_LIBRARY}
    ${Boost_LOG_SETUP_LIBRARY}
    gRPC::grpc++_reflection
)
target_link_libraries(${PROJECT_NAME}_queryImages ${common_link_libs})

install(
  TARGETS ${PROJECT_NAME}_queryImages
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
