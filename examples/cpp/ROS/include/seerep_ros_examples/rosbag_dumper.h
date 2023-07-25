#ifndef ROSBAG_DUMPER
#define ROSBAG_DUMPER

/* ROS specific includes */
#include <ros/master.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_msgs/TFMessage.h>

/* SEEREP storage interfaces */
#include <seerep_hdf5_core/hdf5_core_general.h>
#include <seerep_hdf5_fb/hdf5_fb_image.h>
#include <seerep_hdf5_fb/hdf5_fb_tf.h>

/* ROS to FB conversions*/
#include <seerep_ros_conversions_fb/conversions.h>

/* std */
#include <filesystem>

/* boost */
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace seerep_ros_examples
{
class RosbagDumper
{
public:
  RosbagDumper(const std::filesystem::path& bag_path, const std::filesystem::path& hdf5_path,
               const std::string& project_name, const std::string& project_frame);
  void iterateAndDumpTf(const std::string& tf_topic, const bool is_static = false);
  void iterateAndDumpImage(const std::string& image_topic, const std::string& camera_info_topic);
  void dumpCameraIntrinsics(const std::string& camera_info_topic);
  ~RosbagDumper();

private:
  /* rosbag object to iterate over*/
  rosbag::Bag bag_;

  /* io interface to handle common cases such as writing attributes */
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreGeneral> general_io_;
  /* seerep hdf5 io interface to store camera intrinsics*/
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreCameraIntrinsics> camera_intrinsics_io_;
  // TODO: change to seerep_hdf5_ros storage interface when available
  std::shared_ptr<seerep_hdf5_fb::Hdf5FbImage> image_io_;
  // TODO: change to seerep_hdf5_ros storage interface when available
  std::shared_ptr<seerep_hdf5_fb::Hdf5FbTf> tf_io_;
};
}  // namespace seerep_ros_examples

#endif  // ROSBAG_DUMPER
