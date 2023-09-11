#ifndef SEEREP_GRPC_ROS_ROSBAG_DUMPER
#define SEEREP_GRPC_ROS_ROSBAG_DUMPER

#include <seerep-hdf5-ros/hdf5-ros.h>

#include <chrono>
#include <filesystem>
#include <fstream>

// ros
#include <ros/console.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// logging
#include <boost/log/core.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>

namespace seerep_grpc_ros
{
class RosbagConverterEval
{
public:
  RosbagConverterEval(const std::string& bagPath, const std::string& hdf5FilePath, const std::string& projectFrameId,
                      const std::string& projectName, const std::string& projectUuid, const std::string& topicImage);
  ~RosbagConverterEval();

private:
  void iterateAndDumpImages();

  std::shared_ptr<seerep_hdf5_ros::Hdf5Ros> hdf5ros;

  rosbag::Bag bag;
  std::string bagPath;
  std::string hdf5FilePath;
  std::string projectFrameId;
  std::string projectName;
  std::string projectUuid;

  std::string topicImage;

  std::vector<long long uint> v;
};

}  // namespace seerep_grpc_ros
#endif  // SEEREP_GRPC_ROS_ROSBAG_DUMPER
