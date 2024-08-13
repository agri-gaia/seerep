#ifndef SEEREP_ROS_ROSTOPIC_DUMPER_H_
#define SEEREP_ROS_ROSTOPIC_DUMPER_H_

#include <ros/master.h>
#include <ros/ros.h>
#include <seerep_hdf5_core/hdf5_core_general.h>
#include <seerep_hdf5_core/hdf5_core_image.h>
#include <seerep_hdf5_pb/hdf5_pb_image.h>
#include <seerep_hdf5_pb/hdf5_pb_pointcloud.h>
#include <seerep_hdf5_pb/hdf5_pb_tf.h>
#include <seerep_ros_conversions_pb/conversions.h>
#include <tf2_msgs/TFMessage.h>

#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <functional>
#include <optional>

#include "types.h"

namespace seerep_ros_comm
{
class DumpSensorMsgs
{
public:
  DumpSensorMsgs(std::string hdf5FilePath, std::string project_frame_id,
                 std::string project_name);

  std::optional<ros::Subscriber> getSubscriber(const std::string& message_type,
                                               const std::string& topic);

  void dump(const std_msgs::Header::ConstPtr& msg) const;

  void dump(const sensor_msgs::PointCloud2::ConstPtr& msg) const;

  void dump(const sensor_msgs::Image::ConstPtr& msg) const;

  void dump(const geometry_msgs::Point::ConstPtr& msg) const;

  void dump(const geometry_msgs::Quaternion::ConstPtr& msg) const;

  void dump(const tf2_msgs::TFMessage::ConstPtr& msg) const;

private:
  std::vector<seerep_core_msgs::LabelCategory> m_labelsCategory;

  std::shared_ptr<seerep_hdf5_pb::Hdf5PbTf> m_ioTf;
  std::shared_ptr<seerep_hdf5_pb::Hdf5PbPointCloud> m_ioPointCloud;
  std::shared_ptr<seerep_hdf5_pb::Hdf5PbImage> m_ioImage;
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreImage> m_ioImageCore;
  ros::NodeHandle nh;
};

}  // namespace seerep_ros_comm

#endif  // SEEREP_ROS_ROSTOPIC_DUMPER_H_
