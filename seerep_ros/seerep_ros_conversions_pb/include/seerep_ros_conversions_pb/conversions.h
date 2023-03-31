#ifndef SEEREP_ROS_CONVERSIONS_PB
#define SEEREP_ROS_CONVERSIONS_PB

// ROS messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>

// Seerep proto messages
#include <seerep_msgs/header.pb.h>
#include <seerep_msgs/image.pb.h>
#include <seerep_msgs/point_cloud_2.pb.h>
#include <seerep_msgs/point_field.pb.h>
#include <seerep_msgs/pose_stamped.pb.h>
#include <seerep_msgs/transform.pb.h>
#include <seerep_msgs/transform_stamped.pb.h>
#include <seerep_msgs/vector3.pb.h>
#include <seerep_msgs/vector3_stamped.pb.h>

namespace seerep_ros_conversions_pb
{
/**
 * @brief Converts a ROS std_msgs/Header message to the corresponding
 * Protobuf Header message
 * @param header std_msgs/Header
 * @return Protobuf Header message
 */
seerep::pb::Header toProto(const std_msgs::Header& header);

/**
 * @brief Converts a Protobuf Header message to the corresponding
 * ROS std_msgs/Header message
 * @param header Protobuf Header
 * @return ROS std_msgs/Header
 */
std_msgs::Header toROS(const seerep::pb::Header& header);

/**
 * @brief Converts a ROS sensor_msgs/PointField message to the corresponding
 * Protobuf PointField message
 * @param point_field sensor_msgs/PointField
 * @return Protobuf PointField message
 */
seerep::pb::PointField toProto(const sensor_msgs::PointField& point_field);

/**
 * @brief Converts a Protobuf PointField message to the corresponding
 * ROS sensor_msgs/PointField message
 * @param point_field Protobuf PointField
 * @return ROS std_sensor_msgs/PointField
 */
sensor_msgs::PointField toROS(const seerep::pb::PointField& point_field);

/**
 * @brief Converts a ROS sensor_msgs/PointCloud2 message to the corresponding
 * Protobuf PointCloud2 message
 * @param cloud sensor_msgs/PointCloud2
 * @param optional projectuuid std::string
 * @return Protobuf PointCloud2 message
 */
seerep::pb::PointCloud2 toProto(const sensor_msgs::PointCloud2& cloud, std::string projectuuid = "");

/**
 * @brief Converts a Protobuf PointCloud2 message to the corresponding
 * ROS sensor_msgs/PointCloud2 message
 * @param cloud Protobuf PointCloud2
 * @return ROS std_sensor_msgs/PointCloud2
 */
sensor_msgs::PointCloud2 toROS(const seerep::pb::PointCloud2& cloud);

/**
 * @brief Converts a ROS sensor_msgs/Image message to the corresponding
 * Protobuf Image message
 * @param image sensor_msgs/Image
 * @param optional projectuuid std::string
 * @return Protobuf Image message
 */
seerep::pb::Image toProto(const sensor_msgs::Image& image, std::string projectuuid = "");

/**
 * @brief Converts a Protobuf Image message to the corresponding
 * ROS sensor_msgs/Image message
 * @param image Protobuf Image
 * @return ROS std_sensor_msgs/Image
 */
sensor_msgs::Image toROS(const seerep::pb::Image& image);

/**
 * @brief Converts a ROS geometry_msgs::Point message to the corresponding
 * Protobuf point message
 * @param point geometry_msgs::Point
 * @return Protobuf Point message
 */
seerep::pb::Point toProto(const geometry_msgs::Point& point);

/**
 * @brief Converts a Protobuf point message to the corresponding
 * ROS geometry_msgs::Point message
 * @param point Protobuf Point
 * @return ROS geometry_msgs::Point
 */
geometry_msgs::Point toROS(const seerep::pb::Point& point);

/**
 * @brief Converts a ROS geometry_msgs::Quaternion message to the corresponding
 * Protobuf Quaternion message
 * @param quaternion geometry_msgs::Quaternion
 * @return Protobuf Quaternion message
 */
seerep::pb::Quaternion toProto(const geometry_msgs::Quaternion& quaternion);

/**
 * @brief Converts a Protobuf Quaternion message to the corresponding
 * ROS geometry_msgs::Quaternion message
 * @param quaternion Protobuf Quaternion
 * @return ROS geometry_msgs::Quaternion
 */
geometry_msgs::Quaternion toROS(const seerep::pb::Quaternion& quaternion);

/**
 * @brief Converts a ROS geometry_msgs::Pose message to the corresponding
 * Protobuf Pose message
 * @param pose geometry_msgs::Pose
 * @return Protobuf Pose message
 */
seerep::pb::Pose toProto(const geometry_msgs::Pose& pose);

/**
 * @brief Converts a Protobuf Pose message to the corresponding
 * ROS geometry_msgs::Pose message
 * @param pose Protobuf Pose
 * @return ROS geometry_msgs::Pose
 */
geometry_msgs::Pose toROS(const seerep::pb::Pose& pose);

/**
 * @brief Converts a ROS geometry_msgs::PoseStamped message to the corresponding
 * Protobuf PoseStamped message
 * @param pose geometry_msgs::PoseStamped
 * @return Protobuf PoseStamped message
 */
seerep::pb::PoseStamped toProto(const geometry_msgs::PoseStamped& pose);

/**
 * @brief Converts a Protobuf PoseStamped message to the corresponding
 * ROS geometry_msgs::PoseStamped message
 * @param pose Protobuf PoseStamped
 * @return ROS geometry_msgs::PoseStamped
 */
geometry_msgs::PoseStamped toROS(const seerep::pb::PoseStamped& pose);

/**
 * @brief Converts a ROS geometry_msgs::Vector3 message to the corresponding
 * Protobuf Vector3 message
 * @param vector geometry_msgs::Vector3
 * @return Protobuf Vector3 message
 */
seerep::pb::Vector3 toProto(const geometry_msgs::Vector3& vector);

/**
 * @brief Converts a Protobuf Vector3 message to the corresponding
 * ROS geometry_msgs::Vector3 message
 * @param vector Protobuf Vector3
 * @return ROS geometry_msgs::Vector3
 */
geometry_msgs::Vector3 toROS(const seerep::pb::Vector3& vector);

/**
 * @brief Converts a ROS geometry_msgs::Vector3Stamped message to the corresponding
 * Protobuf Vector3Stamped message
 * @param vector geometry_msgs::Vector3Stamped
 * @return Protobuf Vector3Stamped message
 */
seerep::pb::Vector3Stamped toProto(const geometry_msgs::Vector3Stamped& vector);

/**
 * @brief Converts a Protobuf Vector3Stamped message to the corresponding
 * ROS geometry_msgs::Vector3Stamped message
 * @param vector Protobuf Vector3Stamped
 * @return ROS geometry_msgs::Vector3Stamped
 */
geometry_msgs::Vector3Stamped toROS(const seerep::pb::Vector3Stamped& vector);

/**
 * @brief Converts a ROS geometry_msgs::Transform message to the corresponding
 * Protobuf Transform message
 * @param transform geometry_msgs::Transform
 * @return Protobuf Transform message
 */
seerep::pb::Transform toProto(const geometry_msgs::Transform& transform);

/**
 * @brief Converts a Protobuf Transform message to the corresponding
 * ROS geometry_msgs::Transform message
 * @param transform Protobuf Transform
 * @return ROS geometry_msgs::Transform
 */
geometry_msgs::Transform toROS(const seerep::pb::Transform& transform);

/**
 * @brief Converts a ROS geometry_msgs::TransformStamped message to the corresponding
 * Protobuf TransformStamped message
 * @param transform geometry_msgs::TransformStamped
 * @param optional projectuuid std::string
 * @return Protobuf TransformStamped message
 */
seerep::pb::TransformStamped toProto(const geometry_msgs::TransformStamped& transform, std::string projectuuid = "");

/**
 * @brief Converts a Protobuf TransformStamped message to the corresponding
 * ROS geometry_msgs::TransformStamped message
 * @param transform Protobuf TransformStamped
 * @return ROS geometry_msgs::TransformStamped
 */
geometry_msgs::TransformStamped toROS(const seerep::pb::TransformStamped& transform);
} /* namespace seerep_ros_conversions_pb */

#endif /* SEEREP_ROS_CONVERSIONS_PB */
