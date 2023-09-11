#catkin_lint: ignore critical_var_overwrite

cmake_minimum_required(VERSION 3.0.2)
project(seerep_ros_communication VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC -DBOOST_LOG_DYN_LINK")

find_package(SeerepCom REQUIRED)

find_package(seerep_hdf5_ros REQUIRED)
find_package(SeerepHdf5Core REQUIRED)

find_package(
  catkin REQUIRED
  COMPONENTS topic_tools
             std_msgs
             geometry_msgs
             sensor_msgs
             roscpp
             tf
             rosbag
)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(
  Boost
  COMPONENTS log log_setup
  REQUIRED
)

catkin_package(
  INCLUDE_DIRS
  include
  # LIBRARIES
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  sensor_msgs
  roscpp
  topic_tools
  rosbag
  tf
)

include_directories(
  include
  ${seerep_hdf5_ros_INCLUDE_DIRS}
  ${SeerepCom_INCLUDE_DIRS}
  ${SeerepHdf5Core_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# add_executable(${PROJECT_NAME}_client src/client.cpp)
# add_executable(${PROJECT_NAME}_hdf5dump src/hdf5dump.cpp)
# add_executable(${PROJECT_NAME}_querier src/querier.cpp)
# add_executable(${PROJECT_NAME}_querierFb src/querierFb.cpp)
add_executable(${PROJECT_NAME}_rosbagConverterEval src/rosbagConverterEval.cpp)

# add_executable( ${PROJECT_NAME}_clientImagesWithDetection
# src/clientImagesWithDetection.cpp )

# add_dependencies( ${PROJECT_NAME}_client ${${PROJECT_NAME}_EXPORTED_TARGETS}
# ${catkin_EXPORTED_TARGETS} ) add_dependencies( ${PROJECT_NAME}_hdf5dump
# ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
# add_dependencies( ${PROJECT_NAME}_querier ${${PROJECT_NAME}_EXPORTED_TARGETS}
# ${catkin_EXPORTED_TARGETS} ) add_dependencies( ${PROJECT_NAME}_querierFb
# ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
# add_dependencies( ${PROJECT_NAME}_clientImagesWithDetection
# ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
add_dependencies(
  ${PROJECT_NAME}_rosbagConverterEval ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# target_link_libraries( ${PROJECT_NAME}_client ${SeerepCom_LIBRARIES}
# ${seerep_ros_conversions_pb_LIBRARIES} ${catkin_LIBRARIES}
# gRPC::grpc++_reflection ) target_link_libraries( ${PROJECT_NAME}_hdf5dump
# ${SeerepCom_LIBRARIES} ${seerep_ros_conversions_pb_LIBRARIES}
# ${SeerepHdf5Pb_LIBRARIES} ${SeerepHdf5Core_LIBRARIES} ${catkin_LIBRARIES}
# ${Boost_LIBRARIES} ${Boost_LOG_LIBRARY} ${Boost_LOG_SETUP_LIBRARY} )
# target_link_libraries( ${PROJECT_NAME}_querier ${SeerepCom_LIBRARIES}
# ${seerep_ros_conversions_pb_LIBRARIES} ${catkin_LIBRARIES}
# gRPC::grpc++_reflection ) target_link_libraries( ${PROJECT_NAME}_querierFb
# ${SeerepCom_LIBRARIES} ${seerep_ros_conversions_fb_LIBRARIES}
# ${catkin_LIBRARIES} gRPC::grpc++_reflection ) target_link_libraries(
# ${PROJECT_NAME}_clientImagesWithDetection ${SeerepCom_LIBRARIES}
# ${seerep_ros_conversions_fb_LIBRARIES} ${catkin_LIBRARIES}
# gRPC::grpc++_reflection ${GeographicLib_LIBRARIES} )
target_link_libraries(
  ${PROJECT_NAME}_rosbagConverterEval
  ${SeerepHdf5Ros_LIBRARIES}
  ${SeerepHdf5Core_LIBRARIES}
  ${SeerepCore_LIBRARIES}
  ${seerep_hdf5_ros_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Boost_LOG_LIBRARY}
  ${Boost_LOG_SETUP_LIBRARY}
)

install(
  TARGETS ${PROJECT_NAME}_rosbagConverterEval
          # ${PROJECT_NAME}_clientImagesWithDetection ${PROJECT_NAME}_client
          # ${PROJECT_NAME}_hdf5dump ${PROJECT_NAME}_querier
          # ${PROJECT_NAME}_querierFb
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
