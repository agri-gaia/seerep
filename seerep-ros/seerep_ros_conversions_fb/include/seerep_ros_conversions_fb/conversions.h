#ifndef SEEREP_ROS_CONVERSIONS_FB
#define SEEREP_ROS_CONVERSIONS_FB

// ROS messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>
#include <vision_msgs/Detection2DArray.h>

// seerep flatbuffer messages
#include <seerep-msgs/boundingbox2d_labeled_generated.h>
#include <seerep-msgs/boundingboxes2d_labeled_stamped_generated.h>
#include <seerep-msgs/header_generated.h>
#include <seerep-msgs/image_generated.h>
#include <seerep-msgs/point_cloud_2_generated.h>
#include <seerep-msgs/point_field_generated.h>
#include <seerep-msgs/pose_stamped_generated.h>
#include <seerep-msgs/transform_generated.h>
#include <seerep-msgs/transform_stamped_generated.h>
#include <seerep-msgs/vector3_generated.h>
#include <seerep-msgs/vector3_stamped_generated.h>

// grpc / flatbuffer
#include <flatbuffers/grpc.h>

namespace seerep_ros_conversions_fb
{
/**
 * @brief Converts a ROS std_msgs/Header message to the corresponding
 * Flatbuffer Header message
 * @param header std_msgs/Header
 * @return Flatbuffer Header message
 */
flatbuffers::grpc::Message<seerep::fb::Header> toFlat(const std_msgs::Header& header, std::string projectuuid,
                                                      std::string msguuid);
flatbuffers::Offset<seerep::fb::Header> toFlat(const std_msgs::Header& header, std::string projectuuid,
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
 * Flatbuffer PointField message
 * @param point_field sensor_msgs/PointField
 * @return Flatbuffer PointField message
 */
flatbuffers::grpc::Message<seerep::fb::PointField> toFlat(const sensor_msgs::PointField& point_field);
flatbuffers::Offset<seerep::fb::PointField> toFlat(const sensor_msgs::PointField& point_field,
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
 * Flatbuffer PointCloud2 message
 * @param point_field sensor_msgs/PointCloud2
 * @return Flatbuffer PointCloud2 message
 */
flatbuffers::grpc::Message<seerep::fb::PointCloud2> toFlat(const sensor_msgs::PointCloud2& cloud,
                                                           std::string projectuuid, std::string msguuid);
flatbuffers::Offset<seerep::fb::PointCloud2> toFlat(const sensor_msgs::PointCloud2& cloud, std::string projectuuid,
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
 * Flatbuffer Image message
 * @param image sensor_msgs/Image
 * @return Flatbuffer Image message
 */
flatbuffers::grpc::Message<seerep::fb::Image> toFlat(const sensor_msgs::Image& image, std::string projectuuid,
                                                     std::string msguuid);
flatbuffers::Offset<seerep::fb::Image> toFlat(const sensor_msgs::Image& image, std::string projectuuid,
                                              flatbuffers::grpc::MessageBuilder& builder, std::string msguuid);

/**
 * @brief Converts a Flatbuffer Image message to the corresponding
 * ROS sensor_msgs/Image message
 * @param cloud Flatbuffer Image
 * @return ROS std_sensor_msgs/Image
 */
sensor_msgs::Image toROS(const seerep::fb::Image& image);

/**
 * @brief Converts a ROS geometry_msgs::Point message to the corresponding
 * Flatbuffer Image message
 * @param point geometry_msgs::Point
 * @return Flatbuffer Point message
 */
flatbuffers::grpc::Message<seerep::fb::Point> toFlat(const geometry_msgs::Point& point);
flatbuffers::Offset<seerep::fb::Point> toFlat(const geometry_msgs::Point& point,
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
 * Flatbuffer Quaternion message
 * @param quaternion geometry_msgs::Quaternion
 * @return Flatbuffer Quaternion message
 */
flatbuffers::grpc::Message<seerep::fb::Quaternion> toFlat(const geometry_msgs::Quaternion& quaternion);
flatbuffers::Offset<seerep::fb::Quaternion> toFlat(const geometry_msgs::Quaternion& quaternion,
                                                   flatbuffers::grpc::MessageBuilder& builder);

/**
 * @brief Converts a Flatbuffer Quaternion message to the corresponding
 * ROS geometry_msgs::Quaternion message
 * @param quaternion Flatbuffer Quaternion
 * @return ROS geometry_msgs::Quaternion
 */
geometry_msgs::Quaternion toROS(const seerep::fb::Quaternion& quaternion);

/**
 * @brief Converts a ROS geometry_msgs::Pose message to the corresponding
 * Flatbuffer Pose message
 * @param pose geometry_msgs::Pose
 * @return Flatbuffer Pose message
 */
flatbuffers::grpc::Message<seerep::fb::Pose> toFlat(const geometry_msgs::Pose& pose);
flatbuffers::Offset<seerep::fb::Pose> toFlat(const geometry_msgs::Pose& pose,
                                             flatbuffers::grpc::MessageBuilder& builder);

/**
 * @brief Converts a Flatbuffer Pose message to the corresponding
 * ROS geometry_msgs::Pose message
 * @param pose Flatbuffer Pose
 * @return ROS geometry_msgs::Pose
 */
geometry_msgs::Pose toROS(const seerep::fb::Pose& pose);

/**
 * @brief Converts a ROS geometry_msgs::PoseStamped message to the corresponding
 * Flatbuffer PoseStamped message
 * @param pose geometry_msgs::PoseStamped
 * @return Flatbuffer PoseStamped message
 */
flatbuffers::grpc::Message<seerep::fb::PoseStamped> toFlat(const geometry_msgs::PoseStamped& pose,
                                                           std::string projectuuid);
flatbuffers::Offset<seerep::fb::PoseStamped> toFlat(const geometry_msgs::PoseStamped& pose, std::string projectuuid,
                                                    flatbuffers::grpc::MessageBuilder& builder);

/**
 * @brief Converts a Flatbuffer PoseStamped message to the corresponding
 * ROS geometry_msgs::PoseStamped message
 * @param pose Flatbuffer PoseStamped
 * @return ROS geometry_msgs::PoseStamped
 */
geometry_msgs::PoseStamped toROS(const seerep::fb::PoseStamped& pose);

/**
 * @brief Converts a ROS geometry_msgs::Vector3 message to the corresponding
 * Flatbuffer Vector3 message
 * @param pose geometry_msgs::Vector3
 * @return Flatbuffer Vector3 message
 */
flatbuffers::grpc::Message<seerep::fb::Vector3> toFlat(const geometry_msgs::Vector3& vector);
flatbuffers::Offset<seerep::fb::Vector3> toFlat(const geometry_msgs::Vector3& vector,
                                                flatbuffers::grpc::MessageBuilder& builder);

/**
 * @brief Converts a Flatbuffer Vector3 message to the corresponding
 * ROS geometry_msgs::Vector3 message
 * @param pose Flatbuffer Vector3
 * @return ROS geometry_msgs::Vector3
 */
geometry_msgs::Vector3 toROS(const seerep::fb::Vector3& vector);

/**
 * @brief Converts a ROS geometry_msgs::Vector3Stamped message to the corresponding
 * Flatbuffer Vector3Stamped message
 * @param pose geometry_msgs::Vector3Stamped
 * @return Flatbuffer Vector3Stamped message
 */
flatbuffers::grpc::Message<seerep::fb::Vector3Stamped> toFlat(const geometry_msgs::Vector3Stamped& vector,
                                                              std::string projectuuid);
flatbuffers::Offset<seerep::fb::Vector3Stamped> toFlat(const geometry_msgs::Vector3Stamped& vector,
                                                       std::string projectuuid,
                                                       flatbuffers::grpc::MessageBuilder& builder);

/**
 * @brief Converts a Flatbuffer Vector3Stamped message to the corresponding
 * ROS geometry_msgs::Vector3Stamped message
 * @param pose Flatbuffer Vector3Stamped
 * @return ROS geometry_msgs::Vector3Stamped
 */
geometry_msgs::Vector3Stamped toROS(const seerep::fb::Vector3Stamped& vector);

/**
 * @brief Converts a ROS geometry_msgs::Transform message to the corresponding
 * Flatbuffer Transform message
 * @param pose geometry_msgs::Transform
 * @return Flatbuffer Transform message
 */
flatbuffers::grpc::Message<seerep::fb::Transform> toFlat(const geometry_msgs::Transform& transform);
flatbuffers::Offset<seerep::fb::Transform> toFlat(const geometry_msgs::Transform& transform,
                                                  flatbuffers::grpc::MessageBuilder& builder);

/**
 * @brief Converts a Flatbuffer Transform message to the corresponding
 * ROS geometry_msgs::Transform message
 * @param pose Flatbuffer Transform
 * @return ROS geometry_msgs::Transform
 */
geometry_msgs::Transform toROS(const seerep::fb::Transform& transform);

/**
 * @brief Converts a ROS geometry_msgs::TransformStamped message to the corresponding
 * Flatbuffer TransformStamped message
 * @param pose geometry_msgs::TransformStamped
 * @param projectuuid std::string
 * @return Flatbuffer TransformStamped message
 */
flatbuffers::grpc::Message<seerep::fb::TransformStamped> toFlat(const geometry_msgs::TransformStamped& transform,
                                                                std::string projectuuid);
flatbuffers::Offset<seerep::fb::TransformStamped> toFlat(const geometry_msgs::TransformStamped& transform,
                                                         std::string projectuuid,
                                                         flatbuffers::grpc::MessageBuilder& builder);

/**
 * @brief Converts a Flatbuffer TransformStamped message to the corresponding
 * ROS geometry_msgs::TransformStamped message
 * @param pose Flatbuffer TransformStamped
 * @return ROS geometry_msgs::TransformStamped
 */
geometry_msgs::TransformStamped toROS(const seerep::fb::TransformStamped& transform);

/**
 * @brief Converts a ROS vision_msgs::Detection2DArray message to the corresponding
 * Flatbuffer BoundingBoxes2D_labeled_stamped message
 * @param pose geometry_vision_msgs::Detection2DArray
 * @param projectuuid std::string
 * @return Flatbuffer BoundingBoxes2D_labeled_stamped message
 */
flatbuffers::grpc::Message<seerep::fb::BoundingBoxes2DLabeledStamped>
toFlat(const vision_msgs::Detection2DArray& detection2d, std::string projectuuid, std::string msguuid);
flatbuffers::Offset<seerep::fb::BoundingBoxes2DLabeledStamped> toFlat(const vision_msgs::Detection2DArray& detection2d,
                                                                      std::string projectuuid,
                                                                      flatbuffers::grpc::MessageBuilder& builder,
                                                                      std::string msguuid);

/**
 * @brief Converts a Flatbuffer BoundingBoxes2D_labeled_stamped message to the corresponding
 * ROS vision_msgs::Detection2DArray message
 * @param pose Flatbuffer BoundingBoxes2D_labeled_stamped
 * @return ROS vision_msgs::Detection2DArray
 */
vision_msgs::Detection2DArray toROS(const seerep::fb::BoundingBoxes2DLabeledStamped& bb_labeled_stamped);

/**
 * @brief Converts a ROS vision_msgs::Detection2D message to the corresponding
 * Flatbuffer BoundingBoxes2DLabeled message
 * @param pose geometry_vision_msgs::Detection2D
 * @param projectuuid std::string
 * @return Flatbuffer BoundingBoxes2DLabeled message
 */
flatbuffers::grpc::Message<seerep::fb::BoundingBox2DLabeled> toFlat(const vision_msgs::Detection2D& detection2d,
                                                                    std::string projectuuid, std::string msguuid);
flatbuffers::Offset<seerep::fb::BoundingBox2DLabeled> toFlat(const vision_msgs::Detection2D& detection2d,
                                                             std::string projectuuid,
                                                             flatbuffers::grpc::MessageBuilder& builder,
                                                             std::string msguuid);

/**
 * @brief Converts a Flatbuffer BoundingBoxes2DLabeled message to the corresponding
 * ROS vision_msgs::Detection2D message
 * @param pose Flatbuffer BoundingBoxes2DLabeled
 * @return ROS vision_msgs::Detection2D
 */
vision_msgs::Detection2D toROS(const seerep::fb::BoundingBox2DLabeled& bb_labeled_stamped);

} /* namespace seerep_ros_conversions_fb */

#endif /* SEEREP_ROS_CONVERSIONS_FB */
