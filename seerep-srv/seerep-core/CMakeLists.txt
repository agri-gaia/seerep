cmake_minimum_required(VERSION 3.0.2)
project(seerep-server VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SeerepHDF5 REQUIRED)
find_package(seerep_ros_conversions REQUIRED)
find_package(Protobuf REQUIRED)
find_package(SeerepMsgs REQUIRED)
find_package(HighFive REQUIRED)
find_package(PCL 1.10 REQUIRED )

find_package(catkin REQUIRED
	COMPONENTS
	pcl_conversions
)

include_directories(
	include
	${SeerepMsgs_INCLUDE_DIRS}
	${SeerepHDF5_INCLUDE_DIRS}
	${seerep_ros_conversions_INCLUDE_DIRS}
	${PROTOBUF_INCLUDE_DIR}
	${HighFive_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_library(seerepcore
	src/pointcloud.cpp
)

target_link_libraries(seerepcore
	${SeerepMsgs_LIBRARIES}
	${SeerepHDF5_LIBRARIES}
	${PROTOBUF_LIBRARIES}
	${HighFive_LIBRARIES}
	${PCL_LIBRARIES}
)

set_target_properties(seerepcore PROPERTIES PUBLIC_HEADER "${INSTALL_HEADERS}")

set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

install(TARGETS seerepcore
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
