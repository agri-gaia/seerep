#ifndef AG_PROTO_ROS_CONVERSIONS
#define AG_PROTO_ROS_CONVERSIONS

// ROS messages
#include <std_msgs/Header.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

// Agri-Gaia proto messages
#include <ag_proto_msgs/Header.pb.h>
#include <ag_proto_msgs/PointField.pb.h>
#include <ag_proto_msgs/PointCloud2.pb.h>
#include <ag_proto_msgs/Image.pb.h>

namespace ag_proto_ros
{
  /**
   * @brief Converts a ROS std_msgs/Header message to the corresponding
   * Protobuf Header message
   * @param header std_msgs/Header
   * @return Protobuf Header message
   */
  ag::Header toProto(const std_msgs::Header& header);

  /**
   * @brief Converts a Protobuf Header message to the corresponding
   * ROS std_msgs/Header message
   * @param header Protobuf Header
   * @return ROS std_msgs/Header
   */
  std_msgs::Header toROS(const ag::Header& header);

  /**
   * @brief Converts a ROS sensor_msgs/PointField message to the corresponding
   * Protobuf PointField message
   * @param point_field sensor_msgs/PointField
   * @return Protobuf PointField message
   */
  ag::PointField toProto(const sensor_msgs::PointField& point_field);

  /**
   * @brief Converts a Protobuf PointField message to the corresponding
   * ROS sensor_msgs/PointField message
   * @param point_field Protobuf PointField
   * @return ROS std_sensor_msgs/PointField
   */
  sensor_msgs::PointField toROS(const ag::PointField& point_field);

  /**
   * @brief Converts a ROS sensor_msgs/PointCloud2 message to the corresponding
   * Protobuf PointCloud2 message
   * @param point_field sensor_msgs/PointCloud2
   * @return Protobuf PointCloud2 message
   */
  ag::PointCloud2 toProto(const sensor_msgs::PointCloud2& cloud);

  /**
   * @brief Converts a Protobuf PointCloud2 message to the corresponding
   * ROS sensor_msgs/PointCloud2 message
   * @param cloud Protobuf PointCloud2
   * @return ROS std_sensor_msgs/PointCloud2
   */
  sensor_msgs::PointCloud2 toROS(const ag::PointCloud2& cloud);

  /**
   * @brief Converts a ROS sensor_msgs/Image message to the corresponding
   * Protobuf Image message
   * @param image sensor_msgs/Image
   * @return Protobuf Image message
   */
  ag::Image toProto(const sensor_msgs::Image& image);

  /**
   * @brief Converts a Protobuf Image message to the corresponding
   * ROS sensor_msgs/Image message
   * @param cloud Protobuf Image
   * @return ROS std_sensor_msgs/Image
   */
  sensor_msgs::Image toROS(const ag::Image& image);
} /* namespace ag_proto_ros */

#endif /* AG_PROTO_ROS_CONVERSIONS */