cmake_minimum_required(VERSION 3.10)
project(seerep-msgs VERSION 0.1)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
include(BuildFlatBuffers)
include(FindFlatBuffers)

include(FindProtobuf)

find_package(Protobuf REQUIRED)
find_package(Flatbuffers REQUIRED)

set(MY_PROTO_FILES
	protos/boundingbox_labeled.proto
	protos/boundingbox.proto
	protos/boundingbox2d_labeled.proto
	protos/boundingbox2d.proto
	protos/header.proto
	protos/image.proto
	protos/point_cloud_2.proto
	protos/point_field.proto
	protos/point_stamped.proto
	protos/point.proto
	protos/point2d.proto
	protos/polygon_stamped.proto
	protos/polygon.proto
	protos/pose_stamped.proto
	protos/pose.proto
	protos/project_uuids.proto
	protos/projectCreated.proto
	protos/projectCreation.proto
	protos/quaternion_stamped.proto
	protos/quaternion.proto
	protos/query.proto
	protos/server_response.proto
	protos/time_interval.proto
	protos/transform_stamped.proto
	protos/transform.proto
	protos/twist_stamped.proto
	protos/twist_with_covariance_stamped.proto
	protos/twist_with_covariance.proto
	protos/twist.proto
	protos/vector3_stamped.proto
	protos/vector3.proto
)

set(MY_FBS_FILES
	fbs/boundingbox_labeled.fbs
	fbs/boundingbox.fbs
	fbs/boundingbox2d_labeled.fbs
	fbs/boundingbox2d.fbs
	fbs/header.fbs
	fbs/image.fbs
	fbs/point_cloud_2.fbs
	fbs/point_field.fbs
	fbs/point_stamped.fbs
	fbs/point.fbs
	fbs/point2d.fbs
	fbs/polygon_stamped.fbs
	fbs/polygon.fbs
	fbs/pose_stamped.fbs
	fbs/pose.fbs
	fbs/project_uuids.fbs
	fbs/projectCreated.fbs
	fbs/projectCreation.fbs
	fbs/quaternion_stamped.fbs
	fbs/quaternion.fbs
	fbs/query.fbs
	fbs/server_response.fbs
	fbs/time_interval.fbs
	fbs/timestamp.fbs
	fbs/transform_stamped.fbs
	fbs/transform.fbs
	fbs/twist_stamped.fbs
	fbs/twist_with_covariance_stamped.fbs
	fbs/twist_with_covariance.fbs
	fbs/twist.fbs
	fbs/vector3_stamped.fbs
	fbs/vector3.fbs
)

build_flatbuffers("${MY_FBS_FILES}" "" fbschemas "" "${CMAKE_CURRENT_BINARY_DIR}/fbs" "${CMAKE_CURRENT_BINARY_DIR}/fbs" "" )

add_library(FlatBuffersTarget INTERFACE)
target_include_directories(FlatBuffersTarget INTERFACE ${FLATBUFFERS_INCLUDE_DIR})
add_dependencies(FlatBuffersTarget fbschemas)

get_target_property(FlatBuffers_HEADERS fbschemas GENERATED_HEADERS)
message("Generated FlatBuffers Header Files: " ${FlatBuffers_HEADERS})

PROTOBUF_GENERATE_CPP(
	PROTO_SOURCES PROTO_HEADERS
	${MY_PROTO_FILES}
)

protobuf_generate_python(
	PY_SOURCES
	${MY_PROTO_FILES}
)

add_custom_target(protoPythonTarget ALL
	DEPENDS ${PY_SOURCES}
)

include_directories(
	${CMAKE_BUILD_RPATH}
	${PROTOBUF_INCLUDE_DIR}
)

configure_file(SeerepMsgsConfig.h.in SeerepMsgsConfig.h)

add_library(seerepmsgs
	${PROTO_SOURCES}
)

add_dependencies(seerepmsgs FlatBuffersTarget)

set(INSTALL_HEADERS
	${PROTO_HEADERS}
	${FlatBuffers_HEADERS})

set_target_properties(seerepmsgs PROPERTIES PUBLIC_HEADER "${INSTALL_HEADERS}")

set(CMAKE_INSTALL_INCLUDEDIR include)
set(PROTOBUF_IMPORT_DIR ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

install(TARGETS seerepmsgs
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
	${CMAKE_CURRENT_SOURCE_DIR}/SeerepMsgsConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/SeerepMsgsConfig.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
	PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR PROTOBUF_IMPORT_DIR
)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SeerepMsgsConfigVersion.cmake"
  VERSION "${seerep-msgs_VERSION_MAJOR}.${seerep-msgs_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

# install the configuration file
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/SeerepMsgsConfig.cmake
	${CMAKE_CURRENT_BINARY_DIR}/SeerepMsgsConfigVersion.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SeerepMsgs
)

install(FILES
	"${PROJECT_BINARY_DIR}/SeerepMsgsConfig.h"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

install(DIRECTORY protos/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.proto")

#add PIC gcc flag
SET(GCC_COVERAGE_COMPILE_FLAGS "-fPIC")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
