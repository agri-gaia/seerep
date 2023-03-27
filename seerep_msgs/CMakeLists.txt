cmake_minimum_required(VERSION 3.10)
project(seerep_msgs VERSION 0.1)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
include(BuildFlatBuffers)
include(FindFlatBuffers)
include(FindProtobuf)

find_package(Protobuf REQUIRED)
find_package(Flatbuffers REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED)

set(MY_PROTO_FILES
    protos/boundingbox_labeled.proto
    protos/boundingbox_labeled_with_category.proto
    protos/boundingbox_stamped.proto
    protos/boundingbox.proto
    protos/boundingbox2d_labeled.proto
    protos/boundingbox2d_labeled_with_category.proto
    protos/boundingbox2d_stamped.proto
    protos/boundingbox2d.proto
    protos/frame_infos.proto
    protos/frame_query.proto
    protos/geodetic_coordinates.proto
    protos/header.proto
    protos/image.proto
    protos/label_with_instance.proto
    protos/label.proto
    protos/labels_with_category.proto
    protos/labels_with_instance_with_category.proto
    protos/point_cloud_2.proto
    protos/point_field.proto
    protos/point_stamped.proto
    protos/point.proto
    protos/point2d.proto
    protos/polygon_stamped.proto
    protos/polygon.proto
    protos/pose_stamped.proto
    protos/pose.proto
    protos/project_info.proto
    protos/project_infos.proto
    protos/projectCreation.proto
    protos/quaternion_stamped.proto
    protos/quaternion.proto
    protos/query.proto
    protos/server_response.proto
    protos/time_interval.proto
    protos/timestamp.proto
    protos/transform_stamped.proto
    protos/transform_stamped_query.proto
    protos/transform.proto
    protos/twist_stamped.proto
    protos/twist_with_covariance_stamped.proto
    protos/twist_with_covariance.proto
    protos/twist.proto
    protos/vector3_stamped.proto
    protos/vector3.proto
)

set(MY_FBS_FILES
    fbs/attributes_stamped.fbs
    fbs/boundingbox_labeled.fbs
    fbs/boundingbox_labeled_with_category.fbs
    fbs/boundingbox_stamped.fbs
    fbs/boundingbox.fbs
    fbs/boundingbox2d_labeled.fbs
    fbs/boundingbox2d_labeled_with_category.fbs
    fbs/boundingbox2d_stamped.fbs
    fbs/boundingbox2d.fbs
    fbs/boundingboxes2d_labeled_stamped.fbs
    fbs/datatype.fbs
    fbs/empty.fbs
    fbs/frame_infos.fbs
    fbs/frame_query.fbs
    fbs/geodeticCoordinates.fbs
    fbs/header.fbs
    fbs/image.fbs
    fbs/label_with_instance.fbs
    fbs/label.fbs
    fbs/labels_with_category.fbs
    fbs/labels_with_instance_with_category.fbs
    fbs/point_cloud_2.fbs
    fbs/point_field.fbs
    fbs/point_stamped.fbs
    fbs/point.fbs
    fbs/point2d.fbs
    fbs/polygon_stamped.fbs
    fbs/polygon.fbs
    fbs/pose_stamped.fbs
    fbs/pose.fbs
    fbs/project_info.fbs
    fbs/project_infos.fbs
    fbs/projectCreation.fbs
    fbs/quaternion_stamped.fbs
    fbs/quaternion.fbs
    fbs/query.fbs
    fbs/query_instance.fbs
    fbs/server_response.fbs
    fbs/time_interval.fbs
    fbs/timestamp.fbs
    fbs/transform_stamped.fbs
    fbs/transform_stamped_query.fbs
    fbs/transform.fbs
    fbs/twist_stamped.fbs
    fbs/twist_with_covariance_stamped.fbs
    fbs/twist_with_covariance.fbs
    fbs/twist.fbs
    fbs/union_map_entry.fbs
    fbs/uuids_per_project.fbs
    fbs/uuids_project_pair.fbs
    fbs/vector3_stamped.fbs
    fbs/vector3.fbs
)

set(MY_CORE_MSGS
    ${CMAKE_CURRENT_SOURCE_DIR}/core/aabb.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/dataset_indexable.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/header.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/datatype.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/label_with_instance.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/project_info.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/query_result_project.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/query_result.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/query_tf.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/query.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/timeinterval.h
    ${CMAKE_CURRENT_SOURCE_DIR}/core/timestamp.h
)

add_library(SeerepCoreMsgsTarget INTERFACE)
target_sources(SeerepCoreMsgsTarget INTERFACE ${MY_CORE_MSGS})
target_include_directories(
  SeerepCoreMsgsTarget INTERFACE core/ ${Boost_INCLUDE_DIRS}
)
target_link_libraries(SeerepCoreMsgsTarget ${Boost_LIBRARIES})

build_flatbuffers(
  "${MY_FBS_FILES}"
  ""
  fbschemas
  ""
  "${CMAKE_CURRENT_BINARY_DIR}/fbs"
  "${CMAKE_CURRENT_BINARY_DIR}/fbs"
  ""
)

add_library(FlatBuffersTarget INTERFACE)
target_include_directories(
  FlatBuffersTarget INTERFACE ${FLATBUFFERS_INCLUDE_DIR}
)
add_dependencies(FlatBuffersTarget fbschemas)

get_target_property(FlatBuffers_HEADERS fbschemas GENERATED_HEADERS)

protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS ${MY_PROTO_FILES})

protobuf_generate_python(PY_SOURCES ${MY_PROTO_FILES})

add_custom_target(protoPythonTarget ALL DEPENDS ${PY_SOURCES})

add_custom_command(
  TARGET protoPythonTarget
  POST_BUILD
  COMMAND bash -c "sed -i -E \"s/^import.*_pb2/from seerep.pb &/\" *_pb2*.py"
  COMMAND bash -c "mkdir -p protos/seerep/pb && mv *_pb2.py protos/seerep/pb"
  COMMAND
    bash -c
    "touch protos/seerep/pb/__init__.py && touch protos/seerep/__init__.py"
  VERBATIM
)

include_directories(${CMAKE_BUILD_RPATH} ${PROTOBUF_INCLUDE_DIR})

configure_file(SeerepMsgsConfig.h.in SeerepMsgsConfig.h)

add_library(seerepmsgs ${PROTO_SOURCES})

add_dependencies(seerepmsgs FlatBuffersTarget)
add_dependencies(seerepmsgs SeerepCoreMsgsTarget)

set(INSTALL_HEADERS ${PROTO_HEADERS} ${FlatBuffers_HEADERS})

set_target_properties(seerepmsgs PROPERTIES PUBLIC_HEADER "${INSTALL_HEADERS}")

set(CMAKE_INSTALL_INCLUDEDIR include)
set(PROTOBUF_IMPORT_DIR ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
set(FLATBUFFERS_IMPORT_DIR ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR bin)

install(
  TARGETS seerepmsgs
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/SeerepMsgsConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/SeerepMsgsConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR
            CMAKE_INSTALL_LIBDIR
            PROTOBUF_IMPORT_DIR
            FLATBUFFERS_IMPORT_DIR
)

# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SeerepMsgsConfigVersion.cmake"
  VERSION "${seerep-msgs_VERSION_MAJOR}.${seerep-msgs_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.py"
)

# install the generated typescript sources
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.ts"
)

set(SEEREP_COM_DIR_FB ${CMAKE_CURRENT_BINARY_DIR}/../seerep-com/fbs)

install(
  FILES ${MY_FBS_FILES}/
  PERMISSIONS
    OWNER_READ
    OWNER_WRITE
    OWNER_EXECUTE
    GROUP_READ
    GROUP_WRITE
    WORLD_READ
    WORLD_WRITE
    WORLD_EXECUTE
  DESTINATION ${SEEREP_COM_DIR_FB}
)

# install the configuration file
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/SeerepMsgsConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/SeerepMsgsConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SeerepMsgs
)

install(FILES "${PROJECT_BINARY_DIR}/SeerepMsgsConfig.h"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

# install ProtoBuf proto files
install(
  DIRECTORY protos/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.proto"
)

# install FlatBuffers fbs files
install(
  DIRECTORY fbs/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.fbs"
)

# install seerepCoreMsgs files
install(
  DIRECTORY core/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h"
)

# add PIC gcc flag
set(GCC_COVERAGE_COMPILE_FLAGS "-fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")