#ifndef SEEREP_ROS_HDF5_DUMP_H_
#define SEEREP_ROS_HDF5_DUMP_H_

#include <functional>
#include <optional>

// seerep
#include <seerep_ros_conversions_pb/conversions.h>
#include <seerep-hdf5-pb/hdf5-pb-tf.h>
#include <seerep-hdf5-pb/hdf5-pb-image.h>
#include <seerep-hdf5-pb/hdf5-pb-pointcloud.h>

// uuid
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <boost/lexical_cast.hpp>
#include <boost/functional/hash.hpp>

// ros
#include <ros/ros.h>
#include <ros/master.h>
#include <tf2_msgs/TFMessage.h>

// pkg
#include "types.h"

namespace seerep_grpc_ros
{
class DumpSensorMsgs
{
public:
  DumpSensorMsgs(std::string hdf5FilePath);

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
  std::shared_ptr<seerep_hdf5_pb::Hdf5PbTf> m_ioTf;
  std::shared_ptr<seerep_hdf5_pb::Hdf5PbPointCloud> m_ioPointCloud;
  std::shared_ptr<seerep_hdf5_pb::Hdf5PbImage> m_ioImage;
  ros::NodeHandle nh;
};

}  // namespace seerep_grpc_ros

#endif  // SEEREP_ROS_HDF5_DUMP_H_
