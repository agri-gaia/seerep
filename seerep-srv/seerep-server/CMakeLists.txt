cmake_minimum_required(VERSION 3.0.2)
project(seerep-server VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(SeerepHDF5 REQUIRED)
find_package(HighFive REQUIRED)
message("LIBS" ${SeerepHDF5_LIBRARIES})
message("INCS" ${SeerepHDF5_INCLUDE_DIRS})
message("INCS2" ${HighFive_INCLUDE_DIRS})

include_directories(
	include
	${SeerepHDF5_INCLUDE_DIRS}
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
	${SeerepHDF5_LIBRARIES}
	gRPC::grpc++_reflection
)

target_link_libraries(${PROJECT_NAME}_client
	${SeerepHDF5_LIBRARIES}
	gRPC::grpc++_reflection
)

install(TARGETS ${PROJECT_NAME}_server ${PROJECT_NAME}_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

