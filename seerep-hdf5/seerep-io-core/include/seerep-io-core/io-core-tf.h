#ifndef SEEREP_IO_CORE_IO_CORE_TF_H_
#define SEEREP_IO_CORE_IO_CORE_TF_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-io
#include "seerep-io-core/io-core-general.h"

// ros-msgs (tf)
#include <geometry_msgs/TransformStamped.h>

// std
#include <optional>

#include <boost/geometry.hpp>

namespace seerep_io_core
{
class IoCoreTf : public IoCoreGeneral
{
public:
  IoCoreTf(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  std::optional<std::vector<geometry_msgs::TransformStamped>> readTransformStamped(const std::string& id);
  std::optional<std::vector<std::string>> readTransformStampedFrames(const std::string& id);

private:
  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_TF = "tf";

  inline static const std::string SIZE = "size";
};

}  // namespace seerep_io_core

#endif /* SEEREP_IO_CORE_IO_CORE_TF_H_ */
