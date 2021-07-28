cmake_minimum_required(VERSION 3.10)
project(seerep-com VERSION 0.1)

include(FindProtobuf)
find_package(Protobuf REQUIRED)
find_package(SeerepMsgs REQUIRED)


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
include(ProtobufGenerateGrpcCpp)

list(APPEND PROTOBUF_IMPORT_DIRS ${SeerepMsgs_PROTOBUF_IMPORT_DIRS})
protobuf_generate_cpp(
	PROTO_SOURCES PROTO_HEADERS
	protos/transfer_sensor_msgs.proto
)

protobuf_generate_grpc_cpp(
	GRPC_SOURCES GRPC_HEADERS
	protos/transfer_sensor_msgs.proto
)

message("seerep msgs include dir" ${SeerepMsgs_INCLUDE_DIRS})

include_directories(
  ${CMAKE_BUILD_PATH}
  ${SeerepMsgs_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
)

configure_file(SeerepComConfig.h.in SeerepComConfig.h)

add_library(seerepcom
  ${PROTO_SOURCES}
  ${GRPC_SOURCES}
)

set(INSTALL_HEADERS ${PROTO_HEADERS})
list(APPEND INSTALL_HEADERS ${GRPC_HEADERS})

set_target_properties(seerepcom PROPERTIES PUBLIC_HEADER "${INSTALL_HEADERS}")

set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

install(TARGETS seerepcom
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/SeerepComConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/SeerepComConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SeerepComConfigVersion.cmake"
  VERSION "${seerep-com_VERSION_MAJOR}.${seerep-com_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

# install the configuration file
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/SeerepComConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/SeerepComConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SeerepCom
)

install(FILES
  "${PROJECT_BINARY_DIR}/SeerepComConfig.h"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
