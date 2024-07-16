#ifndef SEEREP_CORE_PB_CAMERAINTRINSICS_H_
#define SEEREP_CORE_PB_CAMERAINTRINSICS_H_

#include <seerep_com/camera_intrinsics_service.grpc.pb.h>

#include <functional>
#include <optional>

// seerep_core_msgs
#include <seerep_msgs/camera_intrinsics.h>

// seerep_core
#include <seerep_core/core.h>

#include "core_pb_conversion.h"

namespace seerep_core_pb
{
class CorePbCameraIntrinsics
{
public:
  CorePbCameraIntrinsics(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CorePbCameraIntrinsics();

  std::optional<seerep::pb::CameraIntrinsics>
  getData(const seerep::pb::CameraIntrinsicsQuery& query);
  boost::uuids::uuid addData(const seerep::pb::CameraIntrinsics& camintrinsics);

private:
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  /** the logger for the logging framework */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level>
      m_logger;
};

}  // namespace seerep_core_pb

#endif  // SEEREP_CORE_PB_TF_H_
