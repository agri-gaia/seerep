#catkin_lint: ignore external_file critical_var_append
cmake_minimum_required(VERSION 3.10)
project(seerep_performance_tests VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -fPIC)

find_package(catkin REQUIRED COMPONENTS sensor_msgs roscpp)

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

catkin_package(
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  sensor_msgs
  roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${mcap_INCLUDE_DIRS}
  ${zstd_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}_ros2mcap src/ros2mcap.cpp src/timer.cpp)
target_link_libraries(
  ${PROJECT_NAME}_ros2mcap
  ${catkin_LIBRARIES}
  ${mcap_LIBRARIES}
  ${zstd_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}_ros2mcap
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
