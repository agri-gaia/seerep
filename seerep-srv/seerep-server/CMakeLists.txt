cmake_minimum_required(VERSION 3.0.2)
project(seerep-server VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(SeerepProtobufIO REQUIRED)
find_package(SeerepCoreIO REQUIRED)
find_package(SeerepCom REQUIRED)
find_package(SeerepCore REQUIRED)
find_package(SeerepPbCore REQUIRED)
find_package(HighFive REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(catkin REQUIRED
	COMPONENTS
	seerep_ros_conversions
)

message("LIBS" ${SeerepCoreIO_LIBRARIES})
message("INCS" ${SeerepCoreIO_INCLUDE_DIRS})
message("INCS2" ${HighFive_INCLUDE_DIRS})

include_directories(
	include
	${SeerepCom_INCLUDE_DIRS}
	${SeerepCore_INCLUDE_DIRS}
	${SeerepPbCore_INCLUDE_DIRS}
	${SeerepProtobufIO_INCLUDE_DIRS}
	${SeerepCoreIO_INCLUDE_DIRS}
	${HighFive_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_server_grpc
	include/${PROJECT_NAME}/meta-operations.h
	src/meta-operations.cpp
	include/${PROJECT_NAME}/receive-sensor-msgs.h
	src/receive-sensor-msgs.cpp
	include/${PROJECT_NAME}/image-service.h
	src/image-service.cpp
	include/${PROJECT_NAME}/point-cloud-service.h
	src/point-cloud-service.cpp
	include/${PROJECT_NAME}/tf-service.h
	src/tf-service.cpp
)

add_executable(${PROJECT_NAME}_server
	src/server.cpp
)

add_executable(${PROJECT_NAME}_client
	src/client.cpp
)

target_link_libraries(${PROJECT_NAME}_server
	${SeerepProtobufIO_LIBRARIES}
	${SeerepCoreIO_LIBRARIES}
	${SeerepCom_LIBRARIES}
	${SeerepCore_LIBRARIES}
	${SeerepPbCore_LIBRARIES}
	gRPC::grpc++_reflection
	${PROJECT_NAME}_server_grpc
	Eigen3::Eigen
)

target_link_libraries(${PROJECT_NAME}_client
	${SeerepProtobufIO_LIBRARIES}
	${SeerepCoreIO_LIBRARIES}
	${SeerepCom_LIBRARIES}
	gRPC::grpc++_reflection
)

install(TARGETS ${PROJECT_NAME}_server ${PROJECT_NAME}_client ${PROJECT_NAME}_server_grpc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
