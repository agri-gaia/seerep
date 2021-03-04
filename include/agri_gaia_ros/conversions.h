#ifndef AGRI_GAIA_ROS_CONVERSIONS
#define AGRI_GAIA_ROS_CONVERSIONS

#include <std_msgs/Header.h>
#include "Header.pb.h"

#include <sensor_msgs/PointField.h>
#include "PointField.pb.h"

#include <sensor_msgs/PointCloud2.h>
#include "PointCloud2.pb.h"

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

} /* namespace agri_gaia_ros */

#endif /* AGRI_GAIA_ROS_CONVERSIONS */