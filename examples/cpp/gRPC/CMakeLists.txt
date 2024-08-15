cmake_minimum_required(VERSION 3.0.2)
project(seerep_cpp_examples VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SeerepCom REQUIRED)
find_package(SeerepMsgs REQUIRED)

find_package(Flatbuffers REQUIRED)
find_package(gRPC CONFIG REQUIRED)

include_directories(
  include
  ${SeerepCom_INCLUDE_DIRS}
  ${SeerepMsgs_INCLUDE_DIRS}
  ${FLATBUFFER_INCLUDE_DIR}
)

add_executable(image_query src/image_query.cpp)
add_executable(tf_query src/tf_query.cpp)

set(link_libs ${SeerepCom_LIBRARIES} ${catkin_LIBRARIES}
              gRPC::grpc++_reflection
)
target_link_libraries(image_query ${link_libs})
target_link_libraries(tf_query ${link_libs})

install(
  TARGETS image_query
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS tf_query
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
