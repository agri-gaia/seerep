cmake_minimum_required(VERSION 3.0.2)
project(seerep_ros_communication VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SeerepCom REQUIRED)

find_package(catkin REQUIRED
	COMPONENTS
	topic_tools
	std_msgs
	geometry_msgs
	sensor_msgs
	roscpp
)
find_package(SeerepIoPb REQUIRED)
find_package(seerep_ros_conversions REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)



catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES
	CATKIN_DEPENDS std_msgs geometry_msgs sensor_msgs roscpp seerep_ros_conversions roscpp topic_tools
  # DEPENDS
)

include_directories(
	include
	${SeerepCom_INCLUDE_DIRS}
	${SeerepMsgs_INCLUDE_DIRS}
	${SeerepIoPb_INCLUDE_DIRS}
	${seerep_ros_conversions_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_server
	src/server.cpp
)

add_executable(${PROJECT_NAME}_client
	src/client.cpp
)
add_executable(${PROJECT_NAME}_hdf5dump
	src/hdf5dump.cpp
)
add_executable(${PROJECT_NAME}_querier
	src/querier.cpp
)

add_dependencies(${PROJECT_NAME}_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_hdf5dump ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_querier ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_server
	${SeerepCom_LIBRARIES}
	${seerep_ros_conversions_LIBRARIES}
	${catkin_LIBRARIES}
	gRPC::grpc++_reflection
)

target_link_libraries(${PROJECT_NAME}_client
	${SeerepCom_LIBRARIES}
	${seerep_ros_conversions_LIBRARIES}
	${catkin_LIBRARIES}
	gRPC::grpc++_reflection
)
target_link_libraries(${PROJECT_NAME}_hdf5dump
	${SeerepCom_LIBRARIES}
	${seerep_ros_conversions_LIBRARIES}
	${SeerepIoPb_LIBRARIES}
	${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_querier
	${SeerepCom_LIBRARIES}
	${seerep_ros_conversions_LIBRARIES}
	${catkin_LIBRARIES}
	gRPC::grpc++_reflection
)

install(TARGETS ${PROJECT_NAME}_server ${PROJECT_NAME}_client ${PROJECT_NAME}_hdf5dump ${PROJECT_NAME}_querier
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
