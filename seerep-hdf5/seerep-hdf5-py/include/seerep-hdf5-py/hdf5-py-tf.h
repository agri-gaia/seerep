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

struct TfTransform
{
public:
  TfTransform(int64_t seconds_new, int32_t nanos_new, const std::string& frame_id_new,
              const std::string& child_frame_id_new, const std::array<double, 3>& translation_new,
              const std::array<double, 4>& rotation_new)
    : seconds(seconds_new)
    , nanos(nanos_new)
    , frame_id(frame_id_new)
    , child_frame_id(child_frame_id_new)
    , translation(translation_new)
    , rotation(rotation_new)
  {
  }

  TfTransform() : TfTransform(0, 0, "", "", { 0, 0, 0 }, { 0, 0, 0, 1 })
  {
  }

  int64_t seconds;
  int32_t nanos;
  std::string frame_id;
  std::string child_frame_id;
  std::array<double, 3> translation;
  std::array<double, 4> rotation;
};

class Hdf5PyTf : public Hdf5PyGeneral
{
public:
  Hdf5PyTf(Hdf5FileWrapper& hdf5_file);

  void writeTransformStamped(const TfTransform& tf);

  std::vector<TfTransform> readTransformStamped(const std::string& frame_id);
};

} /* namespace seerep_hdf5_py */

#endif /* SEEREP_HDF5_PY_HDF5_PY_TF_H_ */
