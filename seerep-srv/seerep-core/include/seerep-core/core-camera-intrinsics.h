#ifndef SEEREP_CORE_CORE_CAMERA_INTRINSICS_H_
#define SEEREP_CORE_CORE_CAMERA_INTRINSICS_H_

// seerep-msgs
#include <seerep-msgs/camera_intrinsics.h>

// seerep-hdf5-core
// #include <seerep-hdf5-core/

namespace seerep_core
{
class CoreCameraIntrinsics
{
public:
  CoreCameraIntrinsics(std::shared_ptr<seerep_hdf5_core::Hdf5CoreCameraIntrinsics> hdf5_io);
  ~CoreCameraIntrinsics();

  std::optional<seerep_core_msgs::camera_intrinsics> getData();
}
}  // namespace seerep_core

#endif  // SEEREP_CORE_CORE_CAMERA_INTRINSICS_H_
