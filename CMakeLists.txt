cmake_minimum_required(VERSION 3.0.2)
project(agri_gaia_msgs)

include(FindProtobuf)
find_package(Protobuf REQUIRED)

include_directories(
	${PROTOBUF_INCLUDE_DIR}
)

PROTOBUF_GENERATE_CPP(
	PROTO_SRC PROTO_HEADER 
	
	proto/Header.proto
	proto/PointField.proto
	proto/PointCloud2.proto
	proto/Image.proto
	proto/Vector3.proto
	proto/Vector3Stamped.proto
	proto/Point.proto
	proto/PointStamped.proto
	proto/Quaternion.proto
	proto/QuaternionStamped.proto
	proto/Pose.proto
	proto/PoseStamped.proto
	proto/Polygon.proto
	proto/PolygonStamped.proto
	proto/Transform.proto
	proto/TransformStamped.proto
	proto/Twist.proto
	proto/TwistStamped.proto
	proto/TwistWithCovariance.proto
	proto/TwistWithCovarianceStamped.proto
)

add_library(${PROJECT_NAME}
	${PROTO_HEADER}
	${PROTO_SRC}
)

set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${PROTO_HEADER}")

set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

install(TARGETS ${PROJECT_NAME} 
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/${PROJECT_NAME}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

#install(FILES ${PROTO_HEADER} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

