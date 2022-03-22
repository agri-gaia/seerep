#ifndef SEEREP_HDF5_FB_HDF5_FB_TF_H_
#define SEEREP_HDF5_FB_HDF5_FB_TF_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include "seerep-hdf5-fb/hdf5-fb-general.h"

// seerep-msgs
#include <seerep-msgs/transform_stamped_generated.h>

// std
#include <optional>

#include <boost/geometry.hpp>

namespace seerep_hdf5_fb
{
class Hdf5FbTf : public Hdf5FbGeneral
{
public:
  Hdf5FbTf(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  void writeTransformStamped(const seerep::fb::TransformStamped& tf);

  std::optional<std::vector<flatbuffers::Offset<seerep::fb::TransformStamped>>>
  readTransformStamped(const std::string& id);
  std::optional<std::vector<std::string>> readTransformStampedFrames(const std::string& id);

private:
  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_TF = "tf";

  inline static const std::string SIZE = "size";
};

}  // namespace seerep_hdf5_fb

#endif /* SEEREP_HDF5_FB_HDF5_FB_TF_H_ */
