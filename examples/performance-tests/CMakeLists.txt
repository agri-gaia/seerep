#catkin_lint: ignore external_file critical_var_append
#catkin_lint: ignore uninstalled_script
cmake_minimum_required(VERSION 3.10)
project(seerep_performance_tests VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -fPIC)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
add_definitions(-DBOOST_LOG_DYN_LINK)

find_package(
  Boost
  COMPONENTS log log_setup
  REQUIRED
)

find_package(catkin REQUIRED COMPONENTS sensor_msgs roscpp seerep_hdf5_ros)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
  message(
    STATUS
      "Downloading conan.cmake from https://github.com/conan-io/cmake-conan"
  )
  file(
    DOWNLOAD
    "https://raw.githubusercontent.com/conan-io/cmake-conan/0.18.1/conan.cmake"
    "${CMAKE_BINARY_DIR}/conan.cmake" TLS_VERIFY ON
  )
endif()

include(${CMAKE_BINARY_DIR}/conan.cmake)

conan_cmake_configure(
  REQUIRES
  mcap/0.9.0@
  GENERATORS
  cmake_find_package
)

conan_cmake_autodetect(settings)

conan_cmake_install(
  PATH_OR_REFERENCE
  .
  BUILD
  missing
  REMOTE
  conancenter
  SETTINGS
  ${settings}
)

find_package(mcap)

catkin_package(CATKIN_DEPENDS sensor_msgs roscpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${mcap_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${zstd_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/analysis.cpp src/message-generation.cpp)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${mcap_LIBRARIES}
  ${Boost_LIBRARIES}
  ${zstd_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
