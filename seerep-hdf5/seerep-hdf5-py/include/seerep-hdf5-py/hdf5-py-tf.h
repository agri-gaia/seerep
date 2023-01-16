#ifndef SEEREP_HDF5_PY_HDF5_PY_TF_H_
#define SEEREP_HDF5_PY_HDF5_PY_TF_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include "seerep-hdf5-core/hdf5-core-tf.h"
#include "seerep-hdf5-py/hdf5-py-general.h"

// seerep-msgs
#include <seerep-msgs/transform_stamped.pb.h>

// std
#include <boost/geometry.hpp>
#include <optional>

namespace seerep_hdf5_py
{
class Hdf5PyTf : public Hdf5PyGeneral
{
public:
  Hdf5PyTf(Hdf5FileWrapper& hdf5_file);

  //   void writeTransformStamped(const seerep::TransformStamped& tf);

  //   std::optional<std::vector<seerep::TransformStamped>> readTransformStamped(const std::string& id);
  //   std::optional<std::vector<std::string>> readTransformStampedFrames(const std::string& id);

  // private:
  //   // datatype group names in hdf5
  //   inline static const std::string HDF5_GROUP_TF = "tf";

  //   inline static const std::string SIZE = "size";
};

} /* namespace seerep_hdf5_py */

#endif /* SEEREP_HDF5_PY_HDF5_PY_TF_H_ */
