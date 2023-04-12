#ifndef SEEREP_CORE_CORE_CAMERA_INTRINSICS_H_
#define SEEREP_CORE_CORE_CAMERA_INTRINSICS_H_

#include <seerep-hdf5-core/hdf5-core-cameraintrinsics.h>

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
  CoreCameraIntrinsics(std::shared_ptr<seerep_hdf5_core::Hdf5CoreCameraIntrinsics> hdf5_io);
  ~CoreCameraIntrinsics();

  /**
   * @brief Save seerep core camera intrinsics
   *
   * @param cameraintrinsics seerep core messages camera intrinsics object
   */
  void addData(const seerep_core_msgs::camera_intrinsics& cameraintrinsics);
  /**
   * @brief Get the seerep core messages camera intrinsics object
   *
   * @param cameraintrinsics_query seerep core camera intrinsics query
   */
  std::optional<seerep_core_msgs::camera_intrinsics>
  getData(const seerep_core_msgs::camera_intrinsics_query& cameraintrinsics_query);

private:
  /** @brief shared pointer to the object handling the HDF5 io for Camera Intrinsics */
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreCameraIntrinsics> m_hdf5_io;
  /** @brief object handling the logging */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};
}  // namespace seerep_core

#endif  // SEEREP_CORE_CORE_CAMERA_INTRINSICS_H_
