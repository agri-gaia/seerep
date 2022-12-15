#ifndef SEEREP_HDF5_ROS_H_
#define SEEREP_HDF5_ROS_H_

// Std
#include <mutex>
#include <string>

// HighFive
#include <highfive/H5Easy.hpp>
#include <highfive/H5File.hpp>

// Boost
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

// ROS
#include "sensor_msgs/Image.h"
#include "std_msgs/Header.h"

// Seerep
#include "seerep-hdf5-core/hdf5-core-general.h"

namespace seerep_hdf5_ros
{
class Hdf5Ros
{
public:
  Hdf5Ros() = delete;
  Hdf5Ros(const std::string& path, const std::string& filename, const std::string& projectFrameId,
          const std::string& projectName);

  bool dumpImage(const sensor_msgs::Image& image) const;
  bool dumpHeader(const std_msgs::Header& header, const std::string& datasetPath) const;

private:
  std::string path_;
  std::string filename_;

  std::shared_ptr<HighFive::File> hdf5File_;
};
}  // namespace seerep_hdf5_ros
#endif /* SEEREP_HDF5_ROS_H_ */
