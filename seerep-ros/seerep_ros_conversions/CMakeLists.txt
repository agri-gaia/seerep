cmake_minimum_required(VERSION 3.0.2)
project(seerep_ros_conversions VERSION 0.1)

find_package(catkin REQUIRED
	COMPONENTS
	std_msgs
	geometry_msgs
	sensor_msgs
)

find_package(SeerepMsgs REQUIRED)

configure_file(include/SeerepRosConversionsConfig.h.in SeerepRosConversionsConfig.h)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES seerep_ros_conversions
	CATKIN_DEPENDS std_msgs geometry_msgs sensor_msgs
  DEPENDS SeerepMsgs
)

include_directories(
	include
	${SeerepMsgs_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/${PROJECT_NAME}/conversions.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES
	"${PROJECT_BINARY_DIR}/SeerepRosConversionsConfig.h"
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
