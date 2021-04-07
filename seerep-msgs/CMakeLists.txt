cmake_minimum_required(VERSION 3.10)
project(ag_proto_msgs VERSION 0.1)

include(FindProtobuf)
find_package(Protobuf REQUIRED)

#find_package(gRPC REQUIRED)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
include(ProtobufGenerateGrpcCpp)


PROTOBUF_GENERATE_CPP(
	PROTO_SOURCES PROTO_HEADERS
	protos/header.proto
	protos/point_field.proto
	protos/point_cloud_2.proto
	protos/image.proto
	protos/vector3.proto
	protos/vector3_stamped.proto
	protos/point.proto
	protos/point_stamped.proto
	protos/quaternion.proto
	protos/quaternion_stamped.proto
	protos/pose.proto
	protos/pose_stamped.proto
	protos/polygon.proto
	protos/polygon_stamped.proto
	protos/transform.proto
	protos/transform_stamped.proto
	protos/twist.proto
	protos/twist_stamped.proto
	protos/twist_with_covariance.proto
	protos/twist_with_covariance_stamped.proto
	protos/server_response.proto

	# grpc
	protos/transfer_sensor_msgs.proto
)

PROTOBUF_GENERATE_GRPC_CPP(
	GRPC_SOURCES GRPC_HEADERS
	protos/transfer_sensor_msgs.proto
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
