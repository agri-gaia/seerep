#ifndef AGRI_GAIA_ROS_CONVERSIONS
#define AGRI_GAIA_ROS_CONVERSIONS

// ROS messages
#include <std_msgs/Header.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

// Agri-Gaia messages
#include <agri_gaia_msgs/Header.pb.h>
#include <agri_gaia_msgs/PointField.pb.h>
#include <agri_gaia_msgs/PointCloud2.pb.h>
#include <agri_gaia_msgs/Image.pb.h>

namespace agri_gaia_ros
{
  /**
   * @brief Converts a ROS std_msgs/Header message to the corresponding
   * Protobuf Header message
   * @param header std_msgs/Header
   * @return Protobuf Header message
   */
  ag::ros::Header toProto(const std_msgs::Header& header);

  /**
   * @brief Converts a Protobuf Header message to the corresponding
   * ROS std_msgs/Header message
   * @param header Protobuf Header
   * @return ROS std_msgs/Header
   */
  std_msgs::Header toROS(const ag::ros::Header& header);

  /**
   * @brief Converts a ROS sensor_msgs/PointField message to the corresponding
   * Protobuf PointField message
   * @param point_field sensor_msgs/PointField
   * @return Protobuf PointField message
   */
  ag::ros::PointField toProto(const sensor_msgs::PointField& point_field);

  /**
   * @brief Converts a Protobuf PointField message to the corresponding
   * ROS sensor_msgs/PointField message
   * @param point_field Protobuf PointField
   * @return ROS std_sensor_msgs/PointField
   */
  sensor_msgs::PointField toROS(const ag::ros::PointField& point_field);

  /**
   * @brief Converts a ROS sensor_msgs/PointCloud2 message to the corresponding
   * Protobuf PointCloud2 message
   * @param point_field sensor_msgs/PointCloud2
   * @return Protobuf PointCloud2 message
   */
  ag::ros::PointCloud2 toProto(const sensor_msgs::PointCloud2& cloud);

  /**
   * @brief Converts a Protobuf PointCloud2 message to the corresponding
   * ROS sensor_msgs/PointCloud2 message
   * @param cloud Protobuf PointCloud2
   * @return ROS std_sensor_msgs/PointCloud2
   */
  sensor_msgs::PointCloud2 toROS(const ag::ros::PointCloud2& cloud);

  /**
   * @brief Converts a ROS sensor_msgs/Image message to the corresponding
   * Protobuf Image message
   * @param image sensor_msgs/Image
   * @return Protobuf Image message
   */
  ag::ros::Image toProto(const sensor_msgs::Image& image);

  /**
   * @brief Converts a Protobuf Image message to the corresponding
   * ROS sensor_msgs/Image message
   * @param cloud Protobuf Image
   * @return ROS std_sensor_msgs/Image
   */
  sensor_msgs::Image toROS(const ag::ros::Image& image);
} /* namespace agri_gaia_ros */

#endif /* AGRI_GAIA_ROS_CONVERSIONS */