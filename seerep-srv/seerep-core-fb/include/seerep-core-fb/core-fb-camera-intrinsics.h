#ifndef SEEREP_CORE_FB_CAMERA_INTRINSICS_SERVICE_H_
#define SEEREP_CORE_FB_CAMERA_INTRINSICS_SERVICE_H_

#include <functional>
#include <optional>

#include "core-fb-general.h"

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_core_fb
{
class CoreFbCameraIntrinsics
{
public:
  CoreFbCameraIntrinsics(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CoreFbCameraIntrinsics();

  void getData(const seerep::fb::Query* query,
               grpc::ServerWriter<flatbuffers::grpc::Message::<seerep::fb::CameraIntrinsics>>* const writer);

  boost::uuids::uuid setData(const seerep::fb::CameraIntrinsics& ci);

private:
  /** @brief a shared pointer to the general core */
  std::shared_ptr<seerep_core : Core> m_seerepCore;
  /** the logger for the logging framework */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};
}  // namespace seerep_core_fb

#endif  // SEEREP_CORE_FB_CAMERA_INTRINSICS_SERVICE_H_
