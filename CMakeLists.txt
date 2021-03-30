cmake_minimum_required(VERSION 3.0.2)
project(ag_grpc_hdf5 VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(AGProtoHDF5 REQUIRED)
find_package(HighFive REQUIRED)
message("LIBS" ${AGProtoHDF5_LIBRARIES})
message("INCS" ${AGProtoHDF5_INCLUDE_DIRS})
message("INCS2" ${HighFive_INCLUDE_DIRS})

include_directories(
	include
	${AGProtoHDF5_INCLUDE_DIRS}
	${HighFive_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_server
	src/server.cpp
)

add_executable(${PROJECT_NAME}_client
	src/client.cpp
)

target_link_libraries(${PROJECT_NAME}_server
	${AGProtoHDF5_LIBRARIES}
	gRPC::grpc++_reflection
)

target_link_libraries(${PROJECT_NAME}_client
	${AGProtoHDF5_LIBRARIES}
	gRPC::grpc++_reflection
)

install(TARGETS ${PROJECT_NAME}_server ${PROJECT_NAME}_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

