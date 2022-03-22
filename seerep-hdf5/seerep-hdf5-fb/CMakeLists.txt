cmake_minimum_required(VERSION 3.10)
project(seerep-hdf5-fb VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(FindHighFive)
include(FindFlatBuffers)

find_package(Flatbuffers REQUIRED)
find_package(SeerepMsgs REQUIRED)
find_package(SeerepHdf5Core REQUIRED)
find_package(HighFive REQUIRED)

configure_file(include/SeerepHdf5FbConfig.h.in SeerepHdf5FbConfig.h)

message("HighFive Include Dirs: ${HighFive_INCLUDE_DIRS} Libraries: ${HighFive_LIBRARIES}")

include_directories(
	include
	${SeerepMsgs_INCLUDE_DIRS}
	${SeerepHdf5Core_INCLUDE_DIRS}
	${FLATBUFFER_INCLUDE_DIR}
	${HighFive_INCLUDE_DIRS}
)

add_library(seerephdf5fb
	src/hdf5-fb-general.cpp
	src/hdf5-fb-tf.cpp
	src/hdf5-fb-image.cpp
)

target_link_libraries(seerephdf5fb
	${SeerepMsgs_LIBRARIES}
	${PROTOBUF_LIBRARIES}
	${HighFive_LIBRARIES}
)

set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

install(TARGETS seerephdf5fb
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
	cmake/SeerepHdf5FbConfig.cmake.in
  	${CMAKE_CURRENT_BINARY_DIR}/SeerepHdf5FbConfig.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
	PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SeerepHdf5FbConfigVersion.cmake"
  VERSION "${seerep_hdf5_fb_VERSION_MAJOR}.${seerep_hdf5_fb_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

# install the configuration file
install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/SeerepHdf5FbConfig.cmake
	${CMAKE_CURRENT_BINARY_DIR}/SeerepHdf5FbConfigVersion.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SeerepHdf5Fb
)

install(FILES
	"${PROJECT_BINARY_DIR}/SeerepHdf5FbConfig.h"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

INSTALL(DIRECTORY include/seerep-hdf5-fb DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(FILES
	cmake/FindHighFive.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SeerepHdf5Fb/Modules
)

#add PIC gcc flag
SET(GCC_COVERAGE_COMPILE_FLAGS "-fPIC")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
