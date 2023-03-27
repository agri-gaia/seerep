#ifndef SEEREP_HDF5_PY_HDF5_PY_TF_H_
#define SEEREP_HDF5_PY_HDF5_PY_TF_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include "seerep-hdf5-core/hdf5-core-tf.h"
#include "seerep-hdf5-py/hdf5-py-general.h"

// seerep-msgs
#include <seerep_msgs/transform_stamped.pb.h>

// std
#include <boost/geometry.hpp>
#include <optional>

namespace seerep_hdf5_py
{

struct TfTransform
{
public:
  TfTransform(int64_t seconds, int32_t nanos, const std::string& frameId, const std::string& childFrameId,
              const std::array<double, 3>& translation, const std::array<double, 4>& rotation)
    : seconds_(seconds)
    , nanos_(nanos)
    , frameId_(frameId)
    , childFrameId_(childFrameId)
    , translation_(translation)
    , rotation_(rotation)
  {
  }

  TfTransform() : TfTransform(0, 0, "", "", { 0, 0, 0 }, { 0, 0, 0, 1 })
  {
  }

  int64_t seconds_;
  int32_t nanos_;
  std::string frameId_;
  std::string childFrameId_;
  std::array<double, 3> translation_;
  std::array<double, 4> rotation_;
};

class Hdf5PyTf : public Hdf5PyGeneral
{
public:
  Hdf5PyTf(Hdf5FileWrapper& hdf5File);

  void writeTransformStamped(const TfTransform& tf);

  std::vector<TfTransform> readTransformStamped(const std::string& frameId);

private:
  std::vector<TfTransform> readGroupTransformStamped(const std::string& tfGroupId);
};

} /* namespace seerep_hdf5_py */

#endif /* SEEREP_HDF5_PY_HDF5_PY_TF_H_ */
