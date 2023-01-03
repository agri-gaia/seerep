#ifndef SEEREP_HDF5_ROS_H_
#define SEEREP_HDF5_ROS_H_

// Std
#include <mutex>
#include <string>

// HighFive
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
class Hdf5Ros : public seerep_hdf5_core::Hdf5CoreGeneral
{
public:
  Hdf5Ros() = delete;
  Hdf5Ros(std::shared_ptr<HighFive::File>& hdf5File, std::shared_ptr<std::mutex>& mutex, const std::string& projectName,
          const std::string& projectFrameId);

  void saveHeader(const std::string& hdf5DataSetPath, const std_msgs::Header& header);
  void saveImage(const sensor_msgs::Image& image);
};
}  // namespace seerep_hdf5_ros
#endif /* SEEREP_HDF5_ROS_H_ */
