cmake_minimum_required(VERSION 3.0.2)
project(seerep-server VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC -DBOOST_LOG_DYN_LINK")

find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(SeerepHdf5Pb REQUIRED)
find_package(SeerepHdf5Fb REQUIRED)
find_package(SeerepHdf5Core REQUIRED)
find_package(SeerepCom REQUIRED)
find_package(SeerepCore REQUIRED)
find_package(SeerepCorePb REQUIRED)
find_package(SeerepCoreFb REQUIRED)
find_package(HighFive REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS program_options log log_setup REQUIRED)

find_package(catkin REQUIRED
	COMPONENTS
	seerep_ros_conversions_pb
	seerep_ros_conversions_fb
)

# message("LIBS" ${SeerepCorePb_LIBRARIES})
# message("INCS" ${SeerepCorePb_INCLUDE_DIRS})
# message("INCS2" ${HighFive_INCLUDE_DIRS})

include_directories(
	include
	${SeerepCom_INCLUDE_DIRS}
	${SeerepCore_INCLUDE_DIRS}
	${SeerepCorePb_INCLUDE_DIRS}
	${SeerepCoreFb_INCLUDE_DIRS}
	${SeerepHdf5Pb_INCLUDE_DIRS}
	${SeerepHdf5Fb_INCLUDE_DIRS}
	${SeerepHdf5Core_INCLUDE_DIRS}
	${HighFive_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}_server_grpc
# 	src/meta-operations.cpp
# 	src/tf-service.cpp
# 	src/image-service.cpp
# 	src/point-cloud-service.cpp
# 	# src/receive-sensor-msgs.cpp
# )

# set(INSTALL_HEADERS
# 	include/${PROJECT_NAME}/meta-operations.h
# 	include/${PROJECT_NAME}/tf-service.h
# 	include/${PROJECT_NAME}/image-service.h
# 	include/${PROJECT_NAME}/point-cloud-service.h
# 	# include/${PROJECT_NAME}/receive-sensor-msgs.h
# )

add_executable(${PROJECT_NAME}_server
	src/server.cpp
	src/pb-meta-operations.cpp
	src/pb-tf-service.cpp
	src/pb-image-service.cpp
	src/pb-point-cloud-service.cpp
	src/fb-meta-operations.cpp
	src/fb-tf-service.cpp
	src/fb-image-service.cpp
	src/fb-instance-service.cpp
)


target_link_libraries(${PROJECT_NAME}_server
	${SeerepHdf5Pb_LIBRARIES}
	${SeerepHdf5Fb_LIBRARIES}
	${SeerepHdf5Core_LIBRARIES}
	${SeerepCom_LIBRARIES}
	${SeerepCore_LIBRARIES}
	${SeerepCorePb_LIBRARIES}
	${SeerepCoreFb_LIBRARIES}
	${Boost_LIBRARIES}
	${Boost_LOG_LIBRARY} ${Boost_LOG_SETUP_LIBRARY}
	gRPC::grpc++_reflection
	# ${PROJECT_NAME}_server_grpc
	Eigen3::Eigen
)


install(TARGETS ${PROJECT_NAME}_server #${PROJECT_NAME}_server_grpc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
