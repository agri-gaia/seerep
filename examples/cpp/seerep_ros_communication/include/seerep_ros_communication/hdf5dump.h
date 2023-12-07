#ifndef SEEREP_ROS_HDF5_DUMP_H_
#define SEEREP_ROS_HDF5_DUMP_H_

#include <functional>
#include <optional>

// seerep
#include <seerep_hdf5_core/hdf5_core_general.h>
#include <seerep_hdf5_core/hdf5_core_image.h>
#include <seerep_hdf5_fb/hdf5_fb_pointcloud.h>
#include <seerep_hdf5_pb/hdf5_pb_image.h>
#include <seerep_hdf5_pb/hdf5_pb_tf.h>
#include <seerep_ros_conversions_fb/conversions.h>
#include <seerep_ros_conversions_pb/conversions.h>

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

// ros
#include <ros/master.h>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>

// pkg
#include "types.h"

namespace seerep_grpc_ros
{
class DumpSensorMsgs
{
public:
  DumpSensorMsgs(std::string hdf5FilePath, std::string project_frame_id, std::string project_name);

  std::optional<ros::Subscriber> getSubscriber(const std::string& message_type, const std::string& topic);

  void dump(const std_msgs::Header::ConstPtr& msg) const;

  void dump(const sensor_msgs::PointCloud2::ConstPtr& msg) const;

  void dump(const sensor_msgs::Image::ConstPtr& msg) const;

  void dump(const geometry_msgs::Point::ConstPtr& msg) const;

  void dump(const geometry_msgs::Quaternion::ConstPtr& msg) const;

  void dump(const geometry_msgs::Pose::ConstPtr& msg) const;

  void dump(const geometry_msgs::PoseStamped::ConstPtr& msg) const;

  void dump(const tf2_msgs::TFMessage::ConstPtr& msg) const;

private:
  std::vector<seerep_core_msgs::LabelsWithInstanceWithCategory> m_labelsWithInstanceWithCategory;

  std::shared_ptr<seerep_hdf5_pb::Hdf5PbTf> m_ioTf;
  std::shared_ptr<seerep_hdf5_fb::Hdf5FbPointCloud> m_ioPointCloud;
  std::shared_ptr<seerep_hdf5_pb::Hdf5PbImage> m_ioImage;
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreImage> m_ioImageCore;
  ros::NodeHandle nh;
};

}  // namespace seerep_grpc_ros

#endif  // SEEREP_ROS_HDF5_DUMP_H_
