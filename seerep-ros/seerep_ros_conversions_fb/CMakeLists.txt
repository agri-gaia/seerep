cmake_minimum_required(VERSION 3.0.2)
project(seerep_ros_conversions_fb VERSION 0.1)

find_package(catkin REQUIRED
	COMPONENTS
	std_msgs
	geometry_msgs
	sensor_msgs
)

find_package(SeerepMsgs REQUIRED)

find_package(gRPC CONFIG REQUIRED)

configure_file(include/SeerepRosConversionsFbConfig.h.in SeerepRosConversionsFbConfig.h)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES seerep_ros_conversions_fb
	CATKIN_DEPENDS std_msgs geometry_msgs sensor_msgs
  DEPENDS SeerepMsgs
)

include_directories(
	include
	${SeerepMsgs_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/${PROJECT_NAME}/conversions.cpp
)

add_executable(ros-to-fb-conversion-test src/seerep_ros_conversions_fb/ros_to_fb_conversion_test.cpp)
add_dependencies(
  ros-to-fb-conversion-test
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ros-to-fb-conversion-test
  GTest::gtest
	GTest::gtest_main
	gmock_main
	${catkin_LIBRARIES}
	${SeerepMsgs_LIBRARIES}
	gRPC::grpc++_reflection
)
add_test(ros-to-fb-conversion-test ros-to-fb-conversion-test)

enable_testing()
find_package(GTest 1.12.0 REQUIRED)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


install(TARGETS ros-to-fb-conversion-test
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES
	"${PROJECT_BINARY_DIR}/SeerepRosConversionsFbConfig.h"
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
