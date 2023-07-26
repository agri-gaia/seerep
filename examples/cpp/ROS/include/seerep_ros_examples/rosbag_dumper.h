#ifndef ROSBAG_DUMPER
#define ROSBAG_DUMPER

/* ROS specific includes */
#include <ros/master.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <tf2_msgs/TFMessage.h>

/* opencv includes */
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

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
  void iterateAndDumpCompressedImage(const std::string& image_topic, const std::string& camera_info_topic);
  void dumpCameraIntrinsics(const std::string& camera_info_topic);

  /* get all topics which a specifc type from the rosbag */
  const std::vector<std::string> getAllTopics(const std::string& topic_type);
  ~RosbagDumper();

private:
  /* Rosbag object to iterate over*/
  rosbag::Bag bag_;

  /* Unique identifier for the genertated SEEREP project.
   * Is also used as the filename of the HDF5 file
   */
  std::string project_uuid_;

  /* IO interface to handle common cases such as writing attributes */
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreGeneral> general_io_;
  /* SEEREP HDF5 IO interface to store camera intrinsics*/
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreCameraIntrinsics> camera_intrinsics_io_;
  // TODO: Change to seerep_hdf5_ros storage interface when available
  std::shared_ptr<seerep_hdf5_fb::Hdf5FbImage> image_io_;
  // TODO: Change to seerep_hdf5_ros storage interface when available
  std::shared_ptr<seerep_hdf5_fb::Hdf5FbTf> tf_io_;
};
}  // namespace seerep_ros_examples

#endif  // ROSBAG_DUMPER
