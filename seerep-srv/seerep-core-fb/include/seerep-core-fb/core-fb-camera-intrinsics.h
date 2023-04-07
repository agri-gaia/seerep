#ifndef SEEREP_CORE_FB_CAMERA_INTRINSICS_SERVICE_H_
#define SEEREP_CORE_FB_CAMERA_INTRINSICS_SERVICE_H_

#include <seerep-com/camera_intrinsics_service.grpc.fb.h>

#include <functional>
#include <optional>

#include "core-fb-conversion.h"
#include "core-fb-general.h"

// seerep-core
#include <seerep-core/core-camera-intrinsics.h>
#include <seerep-core/core.h>

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_core_fb
{
class CoreFbCameraIntrinsics
{
  /**
   * @brief This class is the first layer below a user level call to manipulate camera intrinsics. Calls from this class
   * are directly to the agnostic seerep core.
   *
   */
public:
  /**
   * @brief Construct a new Core Fb Camera Intrinsics object based on the general core
   *
   * @param seerepCore a shared pointer to the general core
   */
  CoreFbCameraIntrinsics(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CoreFbCameraIntrinsics();

  /**
   * @brief Function to query camera intrinsics
   *
   * @param query camera intrinsics query
   * @param writer the writer object used to send the camera intrinsics matching the query directly via gRPC
   */
  void getData(const seerep::fb::cameraIntrinsicsQuery& query,
               grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::CameraIntrinsics>>* const writer);

  /**
   * @brief Add new camera intrinsics to an hdf5 file
   *
   * @param ci seerep flatbuffers camera intrinsics object
   * @return boost::uuids::uuid uuid of the stored camera intrinsics
   */
  boost::uuids::uuid setData(const seerep::fb::CameraIntrinsics& ci);

private:
  /** @brief a shared pointer to the general core */
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  /** the logger for the logging framework */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};
}  // namespace seerep_core_fb

#endif  // SEEREP_CORE_FB_CAMERA_INTRINSICS_SERVICE_H_
