#ifndef SEEREP_SERVER_CAMERA_INTRINSICS_OPERATIONS_H_
#define SEEREP_SERVER_CAMERA_INTRINSICS_OPERATIONS_H_

// seerep
#include <seerep_com/camera_intrinsics_service.grpc.pb.h>
#include <seerep_core/core.h>
#include <seerep_core_pb/core_pb_camera_intrinsics.h>

#include "service_constants.h"
#include "util.hpp"

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_server
{
class PbCameraIntrinsicsService final : public seerep::pb::CameraIntrinsicsService::Service
{
public:
  PbCameraIntrinsicsService(std::shared_ptr<seerep_core::Core> seerepCore);

  grpc::Status TransferCameraIntrinsics(grpc::ServerContext* context, const seerep::pb::CameraIntrinsics* camintrinsics,
                                        seerep::pb::ServerResponse* response) override;
  grpc::Status GetCameraIntrinsics(grpc::ServerContext* context,
                                   const seerep::pb::CameraIntrinsicsQuery* camintrinsicsQuery,
                                   seerep::pb::CameraIntrinsics* response) override;

private:
  std::shared_ptr<seerep_core_pb::CorePbCameraIntrinsics> camIntrinsicsPb;
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_META_OPERATIONS_H_
