#ifndef SEEREP_CORE_CORE_CAMERA_INTRINSICS_H_
#define SEEREP_CORE_CORE_CAMERA_INTRINSICS_H_

#include <seerep-core/core.h>

// seerep-msgs
#include <seerep-msgs/camera_intrinsics.h>
#include <seerep-msgs/camera_intrinsics_query.h>

// seerep-hdf5-core
#include <seerep-hdf5-core/hdf5-core-cameraintrinsics.h>

namespace seerep_core
{
class CoreCameraIntrinsics
{
public:
  CoreCameraIntrinsics(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CoreCameraIntrinsics();

  void addData(const seerep_core_msgs::camera_intrinsics& ci);
  void getData(seerep_core_msgs::camera_intrinsics_query& ci_query, seerep_core_msgs::camera_intrinsics& ci);

private:
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreCameraIntrinsics> getHdf5File(const boost::uuids::uuid& project_uuid);
};
}  // namespace seerep_core

#endif  // SEEREP_CORE_CORE_CAMERA_INTRINSICS_H_
