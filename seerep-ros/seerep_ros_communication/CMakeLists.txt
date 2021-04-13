cmake_minimum_required(VERSION 3.0.2)
project(seerep_ros_communication VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SeerepMsgs REQUIRED)

find_package(catkin REQUIRED
	COMPONENTS
	topic_tools
	std_msgs
	geometry_msgs
	sensor_msgs
	seerep_ros_conversions
	roscpp
)

find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)



catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES
	CATKIN_DEPENDS std_msgs geometry_msgs sensor_msgs roscpp seerep_ros_conversions roscpp
  # DEPENDS
)

include_directories(
	include
	${SeerepMsgs_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_server
	src/server.cpp
)

add_executable(${PROJECT_NAME}_client
	src/client.cpp
)

add_dependencies(${PROJECT_NAME}_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_server
	${catkin_LIBRARIES}
	gRPC::grpc++_reflection
)

target_link_libraries(${PROJECT_NAME}_client
	${catkin_LIBRARIES}
	gRPC::grpc++_reflection
)

install(TARGETS ${PROJECT_NAME}_server ${PROJECT_NAME}_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

