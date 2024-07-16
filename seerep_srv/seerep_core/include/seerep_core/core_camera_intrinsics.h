#ifndef SEEREP_CORE_CORE_CAMERA_INTRINSICS_H_
#define SEEREP_CORE_CORE_CAMERA_INTRINSICS_H_

#include <seerep_hdf5_core/hdf5_core_cameraintrinsics.h>

// seerep-msgs
#include <seerep_msgs/camera_intrinsics.h>
#include <seerep_msgs/camera_intrinsics_query.h>

// seerep-hdf5-core
#include <seerep_hdf5_core/hdf5_core_cameraintrinsics.h>

namespace seerep_core
{
class CoreCameraIntrinsics
{
public:
  CoreCameraIntrinsics(std::shared_ptr<HighFive::File>& file,
                       std::shared_ptr<std::mutex>& write_mtx);
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
   * @param camIntrinsicsUuid [in] UUID of the camera intrinsics to be fetched
   */
  std::optional<seerep_core_msgs::camera_intrinsics>
  getData(boost::uuids::uuid camIntrinsicsUuid);

  /**
   * @brief Check if there exists a camera intrinsics against the provided uuid
   *
   * @param camIntrinsicsUuid UUID of Camera Intrinsics
   * @return true If Camera Intrinsics Exists
   * @return false If Camera Intrinsics Exists
   */
  bool cameraIntrinsicExists(boost::uuids::uuid camIntrinsicsUuid);

private:
  /** @brief shared pointer to the object handling the HDF5 io for Camera Intrinsics */
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreCameraIntrinsics> m_hdf5_io;
  /** @brief object handling the logging */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level>
      m_logger;
};
}  // namespace seerep_core

#endif  // SEEREP_CORE_CORE_CAMERA_INTRINSICS_H_
