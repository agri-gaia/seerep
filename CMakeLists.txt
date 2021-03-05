cmake_minimum_required(VERSION 3.10)
project(AgriGaiaMsgs VERSION 0.1)

include(FindProtobuf)
find_package(Protobuf REQUIRED)

include_directories(
	${PROTOBUF_INCLUDE_DIR}
)

PROTOBUF_GENERATE_CPP(
	PROTO_SRC PROTO_HEADER 
	
	proto/Header.proto
	proto/PointField.proto
	proto/PointCloud2.proto
	proto/Image.proto
	proto/Vector3.proto
	proto/Vector3Stamped.proto
	proto/Point.proto
	proto/PointStamped.proto
	proto/Quaternion.proto
	proto/QuaternionStamped.proto
	proto/Pose.proto
	proto/PoseStamped.proto
	proto/Polygon.proto
	proto/PolygonStamped.proto
	proto/Transform.proto
	proto/TransformStamped.proto
	proto/Twist.proto
	proto/TwistStamped.proto
	proto/TwistWithCovariance.proto
	proto/TwistWithCovarianceStamped.proto
)

configure_file(AgriGaiaMsgsConfig.h.in AgriGaiaMsgsConfig.h)

add_library(${PROJECT_NAME}
	${PROTO_HEADER}
	${PROTO_SRC}
)

set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${PROTO_HEADER}")

set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

install(TARGETS ${PROJECT_NAME} 
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
	${CMAKE_CURRENT_SOURCE_DIR}/AgriGaiaMsgsConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/AgriGaiaMsgsConfig.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
	PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/AgriGaiaMsgsConfigVersion.cmake"
  VERSION "${AgriGaiaMsgs_VERSION_MAJOR}.${AgriGaiaMsgs_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

# install the configuration file
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/AgriGaiaMsgsConfig.cmake
	${CMAKE_CURRENT_BINARY_DIR}/AgriGaiaMsgsConfigVersion.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)


#install(EXPORT AgriGaiaMsgsTargets
#  FILE AgriGaiaMsgsTargets.cmake
#  DESTINATION "lib/cmake/${PROJECT_NAME}"
#)

install(FILES
	"${PROJECT_BINARY_DIR}/AgriGaiaMsgsConfig.h"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

#install(FILES ${PROTO_HEADER} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

