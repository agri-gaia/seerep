#ifndef SEEREP_SERVER_FB_CAMERA_INTRINSICS_SERVICE_H_
#define SEEREP_SERVER_FB_CAMERA_INTRINSICS_SERVICE_H_

// seerep
#include <seerep-com/camera_intrinsics_service.grpc.fb.h>
// #include <seerep-core-fb/core-fb-image.h>
#include <seerep-core/core.h>

// #include "util.hpp"

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_server
{
class FbCameraIntrinsicsService final : public seerep::fb::CameraIntrinsicsService::Service
{
public:
  FbCameraIntrinsicsService(std::shared_ptr<seerep_core::Core> seerepCore);

  grpc::Status
  GetCameraIntrinsics(grpc::ServerContext* context, const flatbuffers::grpc::Message<seerep::fb::Query>* request,
                      grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::CameraIntrinsics>>* writer) override;
  grpc::Status
  TransferCameraIntrinsics(grpc::ServerContext* context,
                           grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::CameraIntrinsics>>* reader,
                           flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response) override;

private:
  // std::shared_ptr<seerep_core_fb::CoreFbImage> imageFb;
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_FB_IMAGE_SERVICE_H_
