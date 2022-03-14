cmake_minimum_required(VERSION 3.10)
project(seerep-hdf5-pb VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(cmake/FindHighFive.cmake)

find_package(Protobuf REQUIRED)
find_package(SeerepMsgs REQUIRED)
find_package(HighFive REQUIRED)


configure_file(include/SeerepHdf5PbConfig.h.in SeerepHdf5PbConfig.h)

include_directories(
	include
	${SeerepMsgs_INCLUDE_DIRS}
	${SeerepIoCore_INCLUDE_DIRS}
	${PROTOBUF_INCLUDE_DIR}
	${HighFive_INCLUDE_DIRS}
)

add_library(seerephdf5pb
	src/hdf5-pb-general.cpp
	src/hdf5-pb-tf.cpp
	src/hdf5-pb-pointcloud.cpp
	src/hdf5-pb-image.cpp
)

target_link_libraries(seerephdf5pb
	${SeerepMsgs_LIBRARIES}
	${SeerepIoCore_LIBRARIES}
	${PROTOBUF_LIBRARIES}
	${HighFive_LIBRARIES}
)

set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

install(TARGETS seerephdf5pb
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
	cmake/SeerepHdf5PbConfig.cmake.in
  	${CMAKE_CURRENT_BINARY_DIR}/SeerepHdf5PbConfig.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
	PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SeerepHdf5PbConfigVersion.cmake"
  VERSION "${seerep_hdf5_pb_VERSION_MAJOR}.${seerep_hdf5_pb_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

# install the configuration file
install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/SeerepHdf5PbConfig.cmake
	${CMAKE_CURRENT_BINARY_DIR}/SeerepHdf5PbConfigVersion.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SeerepHdf5Pb
)

install(FILES
	"${PROJECT_BINARY_DIR}/SeerepHdf5PbConfig.h"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

INSTALL(DIRECTORY include/seerep-hdf5-pb DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(FILES
	cmake/FindHighFive.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SeerepHdf5Pb/Modules
)

#add PIC gcc flag
SET(GCC_COVERAGE_COMPILE_FLAGS "-fPIC")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
