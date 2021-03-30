cmake_minimum_required(VERSION 3.10)
project(ag_proto_hdf5 VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(cmake/FindHighFive.cmake)

find_package(Protobuf REQUIRED)
find_package(AGProtoMsgs REQUIRED)
find_package(HighFive REQUIRED)

configure_file(include/AGProtoHDF5Config.h.in AGProtoHDF5Config.h)

message("HighFive Include Dirs: ${HighFive_INCLUDE_DIRS} Libraries: ${HighFive_LIBRARIES}")

include_directories(
	include
	${AGProtoMsgs_INCLUDE_DIRS}
	${PROTOBUF_INCLUDE_DIR}
	${HighFive_INCLUDE_DIRS}
)

add_library(agprotohdf5
	src/io.cpp
)

target_link_libraries(agprotohdf5
	${AGProtoMsgs_LIBRARIES}
	${PROTOBUF_LIBRARIES}
	${HighFive_LIBRARIES}
)

set(INSTALL_HEADERS include/ag_proto_hdf5/io.h)

set_target_properties(agprotohdf5 PROPERTIES PUBLIC_HEADER "${INSTALL_HEADERS}")

set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

install(TARGETS agprotohdf5
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
	cmake/AGProtoHDF5Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/AGProtoHDF5Config.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
	PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/AGProtoHDF5ConfigVersion.cmake"
  VERSION "${ag_proto_hdf5_VERSION_MAJOR}.${ag_proto_hdf5_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

# install the configuration file
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/AGProtoHDF5Config.cmake
	${CMAKE_CURRENT_BINARY_DIR}/AGProtoHDF5ConfigVersion.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/AGProtoHDF5
)

install(FILES
	"${PROJECT_BINARY_DIR}/AGProtoHDF5Config.h"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

install(FILES
	cmake/FindHighFive.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/AGProtoHDF5/Modules
)


