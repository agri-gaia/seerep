#ifndef SEEREP_HDF5_CORE_HDF5_CORE_CAMERAINTRINSICS_H_
#define SEEREP_HDF5_CORE_HDF5_CORE_CAMERAINTRINSICS_H_

// highfive
#include <highfive/H5File.hpp>

// seerephdf5
#include "seerep-hdf5-core/hdf5-core-general.h"

// seerep-msgs
#include <seerep-msgs/camera_intrinsics.h>

namespace seerep_hdf5_core
{
class Hdf5CoreCameraIntrinsics : public Hdf5CoreGeneral
{
public:
  Hdf5CoreCameraIntrinsics(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);
  std::optional<std::vector<seerep_core_msgs::camera_intrinsics>> readTransformStamped(const std::string& id);

private:
}
}  // namespace seerep_hdf5_core

#endif  // SEEREP_HDF5_CORE_HDF5_CORE_CAMERAINTRINSICS_H_
