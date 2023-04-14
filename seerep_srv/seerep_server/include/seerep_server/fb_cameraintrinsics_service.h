#ifndef SEEREP_SERVER_FB_CAMERA_INTRINSICS_SERVICE_H_
#define SEEREP_SERVER_FB_CAMERA_INTRINSICS_SERVICE_H_

// seerep
#include <seerep_com/camera_intrinsics_service.grpc.fb.h>
#include <seerep_core/core.h>
#include <seerep_core_fb/core_fb_camera_intrinsics.h>
#include <seerep_core_fb/core_fb_conversion.h>

#include "util.hpp"

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_server
{
class FbCameraIntrinsicsService final : public seerep::fb::CameraIntrinsicsService::Service
{
public:
  /**
   * @brief Construct a new Fb Camera Intrinsics Service object
   *
   * @param seerepCore a shared pointer to the general core
   */
  FbCameraIntrinsicsService(std::shared_ptr<seerep_core::Core> seerepCore);

  /**
   * @brief Get the Camera Intrinsics object
   *
   * @param context custom inital and trailing metadata (currently not used)
   * @param request camera intrinsics request object
   * @param writer object used to stream the camera intrinsics
   * @return grpc::Status status of the request. holds success status
   */
  grpc::Status
  GetCameraIntrinsics(grpc::ServerContext* context,
                      const flatbuffers::grpc::Message<seerep::fb::CameraIntrinsicsQuery>* request,
                      grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::CameraIntrinsics>>* writer) override;
  /**
   * @brief Save the Camera Intrinsics object
   *
   * @param context custom inital and trailing metadata (currently not used)
   * @param request flatbuffers camera intrinsics object inside a seerep request object
   * @param response gRPC message to describe the transmission state of the point clouds
   * @return grpc::Status status of the request. holds success status
   */
  grpc::Status TransferCameraIntrinsics(grpc::ServerContext* context,
                                        const flatbuffers::grpc::Message<seerep::fb::CameraIntrinsics>* request,
                                        flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response) override;

private:
  /** @brief a shared pointer to the camera intrinsics fb core */
  std::shared_ptr<seerep_core_fb::CoreFbCameraIntrinsics> ciFbCore;
  /** @brief the logger for the logging framework */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_FB_IMAGE_SERVICE_H_
