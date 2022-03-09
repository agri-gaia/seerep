cmake_minimum_required(VERSION 3.0.2)
project(seerep-pb-core VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(SeerepProtobufIO REQUIRED)
find_package(seerep_ros_conversions REQUIRED)
find_package(Protobuf REQUIRED)
find_package(SeerepMsgs REQUIRED)
find_package(HighFive REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED)

find_package(catkin REQUIRED
	COMPONENTS
	tf2
)

configure_file(include/SeerepPbCoreConfig.h.in SeerepPbCoreConfig.h)

include_directories(
	include
	${SeerepMsgs_INCLUDE_DIRS}
	${SeerepProtobufIO_INCLUDE_DIRS}
	${seerep_ros_conversions_INCLUDE_DIRS}
	${PROTOBUF_INCLUDE_DIR}
	${HighFive_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_library(seereppbcore
	src/tf.cpp
	src/pointcloud.cpp
	src/image.cpp
)

target_link_libraries(seereppbcore
	${SeerepMsgs_LIBRARIES}
	${SeerepProtobufIO_LIBRARIES}
	${seerep_ros_conversions_LIBRARIES}
	${PROTOBUF_LIBRARIES}
	${HighFive_LIBRARIES}
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
	Eigen3::Eigen
)

set(INSTALL_HEADERS
	include/seerep-pb-core/tf.h
	include/seerep-pb-core/pointcloud.h
	include/seerep-pb-core/image.h
)

set_target_properties(seereppbcore PROPERTIES PUBLIC_HEADER "${INSTALL_HEADERS}")

set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

install(TARGETS seereppbcore
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
include(CMakePackageConfigHelpers)

configure_package_config_file(
	cmake/SeerepPbCoreConfig.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/SeerepPbCoreConfig.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
	PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SeerepPbCoreConfigVersion.cmake"
  VERSION "${seerep_pb_core_VERSION_MAJOR}.${seerep_pb_core_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

# install the configuration file
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/SeerepPbCoreConfig.cmake
	${CMAKE_CURRENT_BINARY_DIR}/SeerepPbCoreConfigVersion.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SeerepPbCore
)

install(FILES
	"${PROJECT_BINARY_DIR}/SeerepPbCoreConfig.h"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
