cmake_minimum_required(VERSION 3.10)
project(ag_proto_msgs VERSION 0.1)

include(FindProtobuf)
find_package(Protobuf REQUIRED)

#find_package(gRPC REQUIRED)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
include(ProtobufGenerateGrpcCpp)


PROTOBUF_GENERATE_CPP(
	PROTO_SOURCES PROTO_HEADERS
	protos/Header.proto
	protos/PointField.proto
	protos/PointCloud2.proto
	protos/Image.proto
	protos/Vector3.proto
	protos/Vector3Stamped.proto
	protos/Point.proto
	protos/PointStamped.proto
	protos/Quaternion.proto
	protos/QuaternionStamped.proto
	protos/Pose.proto
	protos/PoseStamped.proto
	protos/Polygon.proto
	protos/PolygonStamped.proto
	protos/Transform.proto
	protos/TransformStamped.proto
	protos/Twist.proto
	protos/TwistStamped.proto
	protos/TwistWithCovariance.proto
	protos/TwistWithCovarianceStamped.proto
	protos/ServerResponse.proto
	protos/SendPointCloud2.proto
)

PROTOBUF_GENERATE_GRPC_CPP(
	GRPC_SOURCES GRPC_HEADERS
	protos/SendPointCloud2.proto
)

include_directories(
	${CMAKE_BUILD_RPATH}
	${PROTOBUF_INCLUDE_DIR}
)

configure_file(AGProtoMsgsConfig.h.in AGProtoMsgsConfig.h)

add_library(agprotomsgs
	${PROTO_SOURCES}
	${GRPC_SOURCES}
)

message("Hallo: ${GRPC_HEADERS}")
message("Hallo: ${PROTO_HEADERS}")

set(INSTALL_HEADERS ${PROTO_HEADERS})
list(APPEND INSTALL_HEADERS ${GRPC_HEADERS})

set_target_properties(agprotomsgs PROPERTIES PUBLIC_HEADER "${INSTALL_HEADERS}")

set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

install(TARGETS agprotomsgs
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
	${CMAKE_CURRENT_SOURCE_DIR}/AGProtoMsgsConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/AGProtoMsgsConfig.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
	PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/AGProtoMsgsConfigVersion.cmake"
  VERSION "${ag_proto_msgs_VERSION_MAJOR}.${ag_proto_msgs_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

# install the configuration file
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/AGProtoMsgsConfig.cmake
	${CMAKE_CURRENT_BINARY_DIR}/AGProtoMsgsConfigVersion.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/AGProtoMsgs
)

install(FILES
	"${PROJECT_BINARY_DIR}/AGProtoMsgsConfig.h"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
