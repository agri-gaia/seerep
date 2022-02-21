#ifndef SEEREP_HDF5_IO_TF_H_
#define SEEREP_HDF5_IO_TF_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-io
#include "seerep-hdf5/general-io.h"

// seerep-msgs
#include <seerep-msgs/transform_stamped.pb.h>

// std
#include <optional>

#include <boost/geometry.hpp>

namespace seerep_hdf5
{
class SeerepHDF5IOTf : public GeneralIO
{
public:
  SeerepHDF5IOTf(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  void writeTransformStamped(const seerep::TransformStamped& tf);

  std::optional<std::vector<seerep::TransformStamped>> readTransformStamped(const std::string& id);
  std::optional<std::vector<std::string>> readTransformStampedFrames(const std::string& id);

private:
  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_TF = "tf";

  inline static const std::string SIZE = "size";
};

} /* namespace seerep_hdf5 */

#endif /* SEEREP_HDF5_IO_TF_H_ */
