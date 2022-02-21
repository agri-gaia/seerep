cmake_minimum_required(VERSION 3.10)
project(seerep-hdf5 VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(cmake/FindHighFive.cmake)

find_package(Protobuf REQUIRED)
find_package(SeerepMsgs REQUIRED)
find_package(HighFive REQUIRED)

configure_file(include/SeerepHDF5Config.h.in SeerepHDF5Config.h)

message("HighFive Include Dirs: ${HighFive_INCLUDE_DIRS} Libraries: ${HighFive_LIBRARIES}")

include_directories(
	include
	${SeerepMsgs_INCLUDE_DIRS}
	${PROTOBUF_INCLUDE_DIR}
	${HighFive_INCLUDE_DIRS}
)

add_library(seerephdf5
	src/io.cpp
	src/ioGeneral.cpp
	src/ioTf.cpp
	src/ioPointCloud.cpp
	src/ioImage.cpp
)

target_link_libraries(seerephdf5
	${SeerepMsgs_LIBRARIES}
	${PROTOBUF_LIBRARIES}
	${HighFive_LIBRARIES}
)

set(INSTALL_HEADERS
	include/seerep-hdf5/io.h
	include/seerep-hdf5/ioGeneral.h
	include/seerep-hdf5/ioTf.h
	include/seerep-hdf5/ioPointCloud.h
	include/seerep-hdf5/ioImage.h
	include/seerep-hdf5/point_cloud2_iterator.hpp
	include/seerep-hdf5/impl/point_cloud2_iterator.hpp
)

set_target_properties(seerephdf5 PROPERTIES PUBLIC_HEADER "${INSTALL_HEADERS}")

set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

install(TARGETS seerephdf5
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
	cmake/SeerepHDF5Config.cmake.in
  	${CMAKE_CURRENT_BINARY_DIR}/SeerepHDF5Config.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
	PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SeerepHDF5ConfigVersion.cmake"
  VERSION "${seerep_hdf5_VERSION_MAJOR}.${seerep_hdf5_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

# install the configuration file
install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/SeerepHDF5Config.cmake
	${CMAKE_CURRENT_BINARY_DIR}/SeerepHDF5ConfigVersion.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SeerepHDF5
)

install(FILES
	"${PROJECT_BINARY_DIR}/SeerepHDF5Config.h"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

install(FILES
	cmake/FindHighFive.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SeerepHDF5/Modules
)

#add PIC gcc flag
SET(GCC_COVERAGE_COMPILE_FLAGS "-fPIC")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
