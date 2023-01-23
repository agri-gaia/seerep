#ifndef SEEREP_HDF5_PB_HDF5_PB_TF_H_
#define SEEREP_HDF5_PB_HDF5_PB_TF_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include <seerep-hdf5-core/hdf5-core-tf.h>

#include "seerep-hdf5-pb/hdf5-pb-general.h"

// seerep-msgs
#include <seerep-msgs/transform_stamped.pb.h>

// std
#include <boost/geometry.hpp>
#include <optional>

namespace seerep_hdf5_pb
{
class Hdf5PbTf : public Hdf5PbGeneral, public seerep_hdf5_core::Hdf5CoreTf
{
public:
  Hdf5PbTf(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  void writeTransformStamped(const seerep::TransformStamped& tf);

  std::optional<std::vector<seerep::TransformStamped>> readTransformStamped(const std::string& id);
  std::optional<std::vector<std::string>> readTransformStampedFrames(const std::string& id);

private:
  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_TF = "tf";

  inline static const std::string SIZE = "size";
};

} /* namespace seerep_hdf5_pb */

#endif /* SEEREP_HDF5_PB_HDF5_PB_TF_H_ */
