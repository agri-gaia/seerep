cmake_minimum_required(VERSION 3.10)
project(seerep-fb-io VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(FindHighFive)
include(FindFlatBuffers)

find_package(Flatbuffers REQUIRED)
find_package(SeerepMsgs REQUIRED)
find_package(HighFive REQUIRED)

configure_file(include/SeerepFlatbuffersIOConfig.h.in SeerepFlatbuffersIOConfig.h)

message("HighFive Include Dirs: ${HighFive_INCLUDE_DIRS} Libraries: ${HighFive_LIBRARIES}")

include_directories(
	include
	${SeerepMsgs_INCLUDE_DIRS}
	${FLATBUFFER_INCLUDE_DIR}
	${HighFive_INCLUDE_DIRS}
)

add_library(seerep-fb-io
src/general-io-fbs.cpp
src/image-io-fbs.cpp
)

target_link_libraries(seerep-fb-io
	${SeerepMsgs_LIBRARIES}
	${PROTOBUF_LIBRARIES}
	${HighFive_LIBRARIES}
)

set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

install(TARGETS seerep-fb-io
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
	cmake/SeerepFlatbuffersIOConfig.cmake.in
  	${CMAKE_CURRENT_BINARY_DIR}/SeerepFlatbuffersIOConfig.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
	PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR
)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SeerepFlatbuffersIOConfigVersion.cmake"
  VERSION "${seerep-fb-io_VERSION_MAJOR}.${seerep-fb-io_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

# install the configuration file
install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/SeerepFlatbuffersIOConfig.cmake
	${CMAKE_CURRENT_BINARY_DIR}/SeerepFlatbuffersIOConfigVersion.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SeerepFlatbuffersIO
)

install(FILES
	"${PROJECT_BINARY_DIR}/SeerepFlatbuffersIOConfig.h"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

INSTALL(DIRECTORY include/seerep-fb-io DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(FILES
	cmake/FindHighFive.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SeerepFlatbuffersIO/Modules
)

#add PIC gcc flag
SET(GCC_COVERAGE_COMPILE_FLAGS "-fPIC")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
