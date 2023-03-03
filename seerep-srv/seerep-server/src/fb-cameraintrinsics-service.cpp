#include "seerep-server/fb-cameraintrinsics-service.h"

namespace seerep_server
{
FbCameraIntrinsicsService::FbCameraIntrinsicsService(std::shared_ptr<seerep_core::Core> seerepCore)
  : ciCore(std::make_shared<seerep_core::CoreCameraIntrinsics>(seerepCore))
{
}
grpc::Status FbCameraIntrinsicsService::GetCameraIntrinsics(
    grpc::ServerContext* context, const flatbuffers::grpc::Message<seerep::fb::cameraIntrinsicsQuery>* request,
    grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::CameraIntrinsics>>* writer)
{
  (void)context;  // ignore this variable without causing warnings
  auto requestRoot = request->GetRoot();

  // TODO check both project and camera intrinsics uuid are set

  seerep_core_msgs::camera_intrinsics_query ciq_coremsg;
  ciq_coremsg = seerep_core_fb::CoreFbConversion::fromFb(*requestRoot);

  seerep_core_msgs::camera_intrinsics ci;
  ciCore->getData(ciq_coremsg, ci);

  return grpc::Status::OK;
}
grpc::Status FbCameraIntrinsicsService::TransferCameraIntrinsics(
    grpc::ServerContext* context, const flatbuffers::grpc::Message<seerep::fb::CameraIntrinsics>* request,
    flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response)
{
  (void)context;  // ignore this variable without causing warnings
  auto requestRoot = request->GetRoot();

  seerep_core_msgs::camera_intrinsics ci;
  ci = seerep_core_fb::CoreFbConversion::fromFb(*requestRoot);

  ciCore->addData(ci);

  return grpc::Status::OK;
}
}  // namespace seerep_server
