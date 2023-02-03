#ifndef SEEREP_HDF5_FB_HDF5_FB_CAMERAINTRINSICS_H_
#define SEEREP_HDF5_FB_HDF5_FB_CAMERAINTRINSICS_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include <seerep-hdf5-core/hdf5-core-tf.h>
#include <seerep-hdf5-fb/hdf5-fb-general.h>

// seerep-msgs
#include <seerep-msgs/camera_intrinsics.h>

// std
#include <optional>

namespace seerep_hdf5_fb
{
class Hdf5FbCameraIntrinsics : public Hdf5FbGeneral
{
public:
  Hdf5FbCameraIntrinsics(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  void writeCameraIntrinsics(const seerep::fb::TransformStamped& tf);

  std::optional<std::vector<flatbuffers::Offset<seerep::fb::CameraIntrinsics>>>
  readCameraIntrinsics(const std::string& id);
}
}  // namespace seerep_hdf5_fb

#endif  // SEEREP_HDF5_FB_HDF5_FB_CAMERAINTRINSICS_H_
