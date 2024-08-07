#ifndef SEEREP_ROS_CONVERSIONS_FB
#define SEEREP_ROS_CONVERSIONS_FB

// ROS messages
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>
#include <vision_msgs/Detection2DArray.h>

// seerep flatbuffer messages
#include <seerep_msgs/camera_intrinsics_generated.h>
#include <seerep_msgs/header_generated.h>
#include <seerep_msgs/image_generated.h>
#include <seerep_msgs/point_cloud_2_generated.h>
#include <seerep_msgs/point_field_generated.h>
#include <seerep_msgs/point_generated.h>
#include <seerep_msgs/transform_generated.h>
#include <seerep_msgs/transform_stamped_generated.h>
#include <seerep_msgs/vector3_generated.h>

// grpc / flatbuffer
#include <flatbuffers/grpc.h>

namespace seerep_ros_conversions_fb
{
/**
 * @brief Converts a ROS std_msgs/Header message to the corresponding
 * gRPC Flatbuffer Header message
 * @param header std_msgs/Header
 * @param projectuuid UUID of the target project. Used to set the corresponding
 * field in the Flatbuffer message
 * @param msguuid UUID of the message. Used to set the corresponding field in
 * the Flatbuffer message
 * @return gRPC Flatbuffer Header message
 */
flatbuffers::grpc::Message<seerep::fb::Header>
toFlat(const std_msgs::Header& header, std::string projectuuid,
       std::string msguuid);
/**
 * @brief Converts a ROS std_msgs/Header message to the corresponding
 * Flatbuffer Header message
 * @param header std_msgs/Header
 * @param projectuuid UUID of the target project. Used to set the corresponding
 * field in the Flatbuffer message
 * @param builder the flatbuffer message builder to build the flatbuffer message
 * @param msguuid UUID of the message. Used to set the corresponding field in
 * the Flatbuffer message
 * @return Flatbuffer Header message
 */
flatbuffers::Offset<seerep::fb::Header>
toFlat(const std_msgs::Header& header, std::string projectuuid,
       flatbuffers::grpc::MessageBuilder& builder, std::string msguuid);

/**
 * @brief Converts a Flatbuffer Header message to the corresponding
 * ROS std_msgs/Header message
 * @param header Flatbuffer Header
 * @return ROS std_msgs/Header
 */
std_msgs::Header toROS(const seerep::fb::Header& header);

/**
 * @brief Converts a ROS sensor_msgs/PointField message to the corresponding
 * gRPC Flatbuffer PointField message
 * @param point_field sensor_msgs/PointField
 * @return gRPC Flatbuffer PointField message
 */
flatbuffers::grpc::Message<seerep::fb::PointField>
toFlat(const sensor_msgs::PointField& point_field);
/**
 * @brief Converts a ROS sensor_msgs/PointField message to the corresponding
 * Flatbuffer PointField message
 * @param point_field sensor_msgs/PointField
 * @param builder the flatbuffer message builder to build the flatbuffer message
 * @return Flatbuffer PointField message
 */
flatbuffers::Offset<seerep::fb::PointField>
toFlat(const sensor_msgs::PointField& point_field,
       flatbuffers::grpc::MessageBuilder& builder);

/**
 * @brief Converts a Flatbuffer PointField message to the corresponding
 * ROS sensor_msgs/PointField message
 * @param point_field Flatbuffer PointField
 * @return ROS std_sensor_msgs/PointField
 */
sensor_msgs::PointField toROS(const seerep::fb::PointField& point_field);

/**
 * @brief Converts a ROS sensor_msgs/PointCloud2 message to the corresponding
 * gRPC Flatbuffer PointCloud2 message
 * @param cloud sensor_msgs/PointCloud2
 * @param projectuuid UUID of the target project. Used to set the corresponding
 * field in the Flatbuffer message
 * @param msguuid UUID of the message. Used to set the corresponding field in
 * the Flatbuffer message
 * @return gRPC Flatbuffer PointCloud2 message
 */
flatbuffers::grpc::Message<seerep::fb::PointCloud2>
toFlat(const sensor_msgs::PointCloud2& cloud, std::string projectuuid,
       std::string msguuid);
/**
 * @brief Converts a ROS sensor_msgs/PointCloud2 message to the corresponding
 * Flatbuffer PointCloud2 message
 * @param cloud sensor_msgs/PointCloud2
 * @param projectuuid UUID of the target project. Used to set the corresponding
 * field in the Flatbuffer message
 * @param builder the flatbuffer message builder to build the flatbuffer message
 * @param msguuid UUID of the message. Used to set the corresponding field in
 * the Flatbuffer message
 * @return Flatbuffer PointCloud2 message
 */
flatbuffers::Offset<seerep::fb::PointCloud2>
toFlat(const sensor_msgs::PointCloud2& cloud, std::string projectuuid,
       flatbuffers::grpc::MessageBuilder& builder, std::string msguuid);

/**
 * @brief Converts a Flatbuffer PointCloud2 message to the corresponding
 * ROS sensor_msgs/PointCloud2 message
 * @param cloud Flatbuffer PointCloud2
 * @return ROS std_sensor_msgs/PointCloud2
 */
sensor_msgs::PointCloud2 toROS(const seerep::fb::PointCloud2& cloud);

/**
 * @brief Converts a ROS sensor_msgs/Image message to the corresponding
 * gRPC Flatbuffer Image message
 * @param image sensor_msgs/Image
 * @param projectuuid UUID of the target project. Used to set the corresponding
 * field in the Flatbuffer message
 * @param msguuid UUID of the message. Used to set the corresponding field in
 * the Flatbuffer message
 * @return gRPC Flatbuffer Image message
 */
flatbuffers::grpc::Message<seerep::fb::Image>
toFlat(const sensor_msgs::Image& image, std::string projectuuid,
       std::string cameraInstrinsicUuid, std::string msguuid = "");
/**
 * @brief Converts a ROS sensor_msgs/Image message to the corresponding
 * Flatbuffer Image message
 * @param image sensor_msgs/Image
 * @param projectuuid UUID of the target project. Used to set the corresponding
 * field in the Flatbuffer message
 * @param builder the flatbuffer message builder to build the flatbuffer message
 * @param msguuid UUID of the message. Used to set the corresponding field in
 * the Flatbuffer message
 * @return Flatbuffer Image message
 */
flatbuffers::Offset<seerep::fb::Image>
toFlat(const sensor_msgs::Image& image, std::string projectuuid,
       flatbuffers::grpc::MessageBuilder& builder,
       std::string cameraInstrinsicUuid, std::string msguuid = "");

/**
 * @brief Converts a Flatbuffer Image message to the corresponding
 * ROS sensor_msgs/Image message
 * @param image Flatbuffer Image
 * @return ROS std_sensor_msgs/Image
 */
sensor_msgs::Image toROS(const seerep::fb::Image& image);

/**
 * @brief Converts a ROS geometry_msgs::Point message to the corresponding
 * gRPC Flatbuffer Image message
 * @param point geometry_msgs::Point
 * @return gRPC Flatbuffer Point message
 */
flatbuffers::grpc::Message<seerep::fb::Point>
toFlat(const geometry_msgs::Point& point);
/**
 * @brief Converts a ROS geometry_msgs::Point message to the corresponding
 * Flatbuffer Image message
 * @param point geometry_msgs::Point
 * @param builder the flatbuffer message builder to build the flatbuffer message
 * @return Flatbuffer Point message
 */
flatbuffers::Offset<seerep::fb::Point>
toFlat(const geometry_msgs::Point& point,
       flatbuffers::grpc::MessageBuilder& builder);

/**
 * @brief Converts a Flatbuffer Image message to the corresponding
 * ROS geometry_msgs::Point message
 * @param point Flatbuffer Point
 * @return ROS geometry_msgs::Point
 */
geometry_msgs::Point toROS(const seerep::fb::Point& point);

/**
 * @brief Converts a ROS geometry_msgs::Quaternion message to the corresponding
 * gRPC Flatbuffer Quaternion message
 * @param quaternion geometry_msgs::Quaternion
 * @return gRPC Flatbuffer Quaternion message
 */
flatbuffers::grpc::Message<seerep::fb::Quaternion>
toFlat(const geometry_msgs::Quaternion& quaternion);
/**
 * @brief Converts a ROS geometry_msgs::Quaternion message to the corresponding
 * Flatbuffer Quaternion message
 * @param quaternion geometry_msgs::Quaternion
 * @param builder the flatbuffer message builder to build the flatbuffer message
 * @return Flatbuffer Quaternion message
 */
flatbuffers::Offset<seerep::fb::Quaternion>
toFlat(const geometry_msgs::Quaternion& quaternion,
       flatbuffers::grpc::MessageBuilder& builder);

/**
 * @brief Converts a Flatbuffer Quaternion message to the corresponding
 * ROS geometry_msgs::Quaternion message
 * @param quaternion Flatbuffer Quaternion
 * @return ROS geometry_msgs::Quaternion
 */
geometry_msgs::Quaternion toROS(const seerep::fb::Quaternion& quaternion);

/**
 * @brief Converts a ROS geometry_msgs::Vector3 message to the corresponding
 * gRPC Flatbuffer Vector3 message
 * @param vector geometry_msgs::Vector3
 * @return gRPC Flatbuffer Vector3 message
 */
flatbuffers::grpc::Message<seerep::fb::Vector3>
toFlat(const geometry_msgs::Vector3& vector);
/**
 * @brief Converts a ROS geometry_msgs::Vector3 message to the corresponding
 * Flatbuffer Vector3 message
 * @param vector geometry_msgs::Vector3
 * @param builder the flatbuffer message builder to build the flatbuffer message
 * @return Flatbuffer Vector3 message
 */
flatbuffers::Offset<seerep::fb::Vector3>
toFlat(const geometry_msgs::Vector3& vector,
       flatbuffers::grpc::MessageBuilder& builder);

/**
 * @brief Converts a Flatbuffer Vector3 message to the corresponding
 * ROS geometry_msgs::Vector3 message
 * @param vector Flatbuffer Vector3
 * @return ROS geometry_msgs::Vector3
 */
geometry_msgs::Vector3 toROS(const seerep::fb::Vector3& vector);

/**
 * @brief Converts a ROS geometry_msgs::Transform message to the corresponding
 * gRPC Flatbuffer Transform message
 * @param transform geometry_msgs::Transform
 * @return gRPC Flatbuffer Transform message
 */
flatbuffers::grpc::Message<seerep::fb::Transform>
toFlat(const geometry_msgs::Transform& transform);
/**
 * @brief Converts a ROS geometry_msgs::Transform message to the corresponding
 * Flatbuffer Transform message
 * @param transform geometry_msgs::Transform
 * @param builder the flatbuffer message builder to build the flatbuffer message
 * @return Flatbuffer Transform message
 */
flatbuffers::Offset<seerep::fb::Transform>
toFlat(const geometry_msgs::Transform& transform,
       flatbuffers::grpc::MessageBuilder& builder);

/**
 * @brief Converts a Flatbuffer Transform message to the corresponding
 * ROS geometry_msgs::Transform message
 * @param transform Flatbuffer Transform
 * @return ROS geometry_msgs::Transform
 */
geometry_msgs::Transform toROS(const seerep::fb::Transform& transform);

/**
 * @brief Converts a ROS geometry_msgs::TransformStamped message to the
 * corresponding gRPC Flatbuffer TransformStamped message
 * @param transform geometry_msgs::TransformStamped
 * @param projectuuid std::string
 * @return gRPC Flatbuffer TransformStamped message
 */
flatbuffers::grpc::Message<seerep::fb::TransformStamped>
toFlat(const geometry_msgs::TransformStamped& transform,
       std::string projectuuid, const bool isStatic);
/**
 * @brief Converts a ROS geometry_msgs::TransformStamped message to the
 * corresponding Flatbuffer TransformStamped message
 * @param transform geometry_msgs::TransformStamped
 * @param projectuuid std::string
 * @param builder the flatbuffer message builder to build the flatbuffer message
 * @return Flatbuffer TransformStamped message
 */
flatbuffers::Offset<seerep::fb::TransformStamped>
toFlat(const geometry_msgs::TransformStamped& transform,
       std::string projectuuid, const bool isStatic,
       flatbuffers::grpc::MessageBuilder& builder);

/**
 * @brief Converts a Flatbuffer TransformStamped message to the corresponding
 * ROS geometry_msgs::TransformStamped message
 * @param transform Flatbuffer TransformStamped
 * @return ROS geometry_msgs::TransformStamped
 */
geometry_msgs::TransformStamped
toROS(const seerep::fb::TransformStamped& transform);

/**
 * @brief Converts a ROS sensor_msgs::CameraInfo message to the corresponding
 * gRPC Flatbuffer CameraIntrinsics message
 * @param ci sensor_msgs::CameraInfo
 * @return gRPC Flatbuffer CameraIntrinsics message
 */
flatbuffers::grpc::Message<seerep::fb::CameraIntrinsics>
toFlat(const sensor_msgs::CameraInfo& ci, std::string& projectuuid,
       std::string& msgUuid, double maxViewingDistance);
/**
 * @brief Converts a ROS sensor_msgs::CameraInfo message to the corresponding
 * gRPC Flatbuffer CameraIntrinsics message
 * @param ci sensor_msgs::CameraInfo
 * @param builder the flatbuffer message builder to build the flatbuffer message
 * @return Flatbuffer CameraIntrinsics message
 */
flatbuffers::Offset<seerep::fb::CameraIntrinsics>
toFlat(const sensor_msgs::CameraInfo& ci, std::string& projectuuid,
       std::string& msgUuid, double maxViewingDistance,

       flatbuffers::grpc::MessageBuilder& builder);
/**
 * @brief Converts a Flatbuffer CameraIntrinsics message to the corresponding
 * ROS sensor_msgs::CameraInfo message
 * @param ci Flatbuffer CameraIntrinsics message
 * @return ROS sensor_msgs::CameraInfo
 */
sensor_msgs::CameraInfo toROS(const seerep::fb::CameraIntrinsics& ci);
/**
 * @brief Converts a ROS sensor_msgs::RegionOfInterest message to the
 * corresponding gRPC Flatbuffer RegionOfInterest message
 * @param roi sensor_msgs::RegionOfInterest
 * @param builder the flatbuffer message builder to build the flatbuffer message
 * @return Flatbuffer RegionOfInterest message
 */
flatbuffers::Offset<seerep::fb::RegionOfInterest>
toFlat(const sensor_msgs::RegionOfInterest& roi,

       flatbuffers::grpc::MessageBuilder& builder);
/**
 * @brief Converts a Flatbuffer CameraIntrinsics message to the corresponding
 * ROS sensor_msgs::CameraInfo message
 * @param roi Flatbuffer RegionOfInterest message
 * @return ROS sensor_msgs::RegionOfInterest
 */
sensor_msgs::RegionOfInterest toROS(const seerep::fb::RegionOfInterest& roi);
} /* namespace seerep_ros_conversions_fb */

#endif /* SEEREP_ROS_CONVERSIONS_FB */
