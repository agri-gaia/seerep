cmake_minimum_required(VERSION 3.10)
project(seerep_hdf5_py VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC -DBOOST_LOG_DYN_LINK")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS pybind11_catkin)

include(cmake/FindHighFive.cmake)

find_package(Protobuf REQUIRED)
find_package(SeerepMsgs REQUIRED)
find_package(SeerepHdf5Core REQUIRED)
find_package(HighFive REQUIRED)
find_package(gRPC CONFIG REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(
  Boost
  COMPONENTS log log_setup
  REQUIRED
)

enable_testing()
find_package(GTest 1.12.0 REQUIRED)

configure_file(include/SeerepHdf5PyConfig.h.in SeerepHdf5PyConfig.h)

include_directories(
  include
  ${SeerepMsgs_INCLUDE_DIRS}
  ${SeerepHdf5Core_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${HighFive_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

pybind11_add_module(
  seerephdf5py
  src/hdf5-py-general.cpp
  src/hdf5-py-tf.cpp
  src/hdf5-py-pointcloud.cpp
  src/hdf5-py-image.cpp
)

target_link_libraries(
  seerephdf5py
  PRIVATE ${SeerepMsgs_LIBRARIES}
          ${SeerepHdf5Core_LIBRARIES}
          ${SeerepIoCore_LIBRARIES}
          ${PROTOBUF_LIBRARIES}
          ${HighFive_LIBRARIES}
          ${Boost_LIBRARIES}
          ${Boost_LOG_LIBRARY}
          ${Boost_LOG_SETUP_LIBRARY}
)

# test ###

# add_executable(python-write-load-test test/py-write-load-test.cpp)
# add_dependencies( python-write-load-test ${${PROJECT_NAME}_EXPORTED_TARGETS}
# ${catkin_EXPORTED_TARGETS} )

# target_link_libraries( python-write-load-test GTest::gtest GTest::gtest_main
# gmock_main ${catkin_LIBRARIES} ${HighFive_LIBRARIES} gRPC::grpc++_reflection
# seerephdf5py ) add_test(python-write-load-test python-write-load-test)

# install ###

set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

# install( TARGETS python-write-load-test LIBRARY DESTINATION
# ${CMAKE_INSTALL_LIBDIR} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
# PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME} )

install(
  TARGETS seerephdf5py
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

# install( TARGETS seerephdf5py COMPONENT python LIBRARY DESTINATION
# ${PYTHON_LIBRARY_DIR} )

install(DIRECTORY include/seerep-hdf5-py
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES cmake/FindHighFive.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SeerepHdf5Py/Modules
)
