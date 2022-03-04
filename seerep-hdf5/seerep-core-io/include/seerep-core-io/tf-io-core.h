#ifndef SEEREP_CORE_IO_TF_IO_CORE_H_
#define SEEREP_CORE_IO_TF_IO_CORE_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-io
#include "seerep-core-io/general-io-core.h"

// ros-msgs (tf)
#include <geometry_msgs/TransformStamped.h>

// std
#include <optional>

#include <boost/geometry.hpp>

namespace seerep_core_io
{
class TfIOCore : public GeneralIOCore
{
public:
  TfIOCore(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  std::optional<std::vector<geometry_msgs::TransformStamped>> readTransformStamped(const std::string& id);
  std::optional<std::vector<std::string>> readTransformStampedFrames(const std::string& id);

private:
  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_TF = "tf";

  inline static const std::string SIZE = "size";
};

}  // namespace seerep_core_io

#endif /* SEEREP_CORE_IO_TF_IO_CORE_H_ */
