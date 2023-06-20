cmake_minimum_required(VERSION 3.0.2)
project(seerep_ros_conversions_fb VERSION 0.1)

find_package(
  catkin REQUIRED
  COMPONENTS std_msgs
             geometry_msgs
             sensor_msgs
             vision_msgs
)

find_package(SeerepMsgs REQUIRED)
find_package(SeerepCore REQUIRED)

find_package(gRPC CONFIG REQUIRED)

find_package(GTest 1.12.0 REQUIRED)

configure_file(
  include/SeerepRosConversionsFbConfig.h.in SeerepRosConversionsFbConfig.h
)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  seerep_ros_conversions_fb
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  sensor_msgs
  vision_msgs
  DEPENDS
  SeerepMsgs
)

include_directories(include ${SeerepMsgs_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/conversions.cpp)
add_dependencies(
  ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME} ${SeerepCore_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test test/ros_to_fb_conversion_test.cpp)
  target_link_libraries(
    ${PROJECT_NAME}-test
    GTest::gtest
    GTest::gtest_main
    gmock_main
    ${catkin_LIBRARIES}
    ${SeerepMsgs_LIBRARIES}
    gRPC::grpc++_reflection
    ${PROJECT_NAME}
  )
endif(CATKIN_ENABLE_TESTING)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
)

install(FILES "${PROJECT_BINARY_DIR}/SeerepRosConversionsFbConfig.h"
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
