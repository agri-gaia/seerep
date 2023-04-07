#include "seerep-server/fb-cameraintrinsics-service.h"

namespace seerep_server
{
FbCameraIntrinsicsService::FbCameraIntrinsicsService(std::shared_ptr<seerep_core::Core> seerepCore)
  : ciFbCore(std::make_shared<seerep_core_fb::CoreFbCameraIntrinsics>(seerepCore))
{
}
grpc::Status FbCameraIntrinsicsService::GetCameraIntrinsics(
    grpc::ServerContext* context, const flatbuffers::grpc::Message<seerep::fb::cameraIntrinsicsQuery>* request,
    grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::CameraIntrinsics>>* writer)
{
  (void)context;  // ignore this variable without causing warnings
  auto requestRoot = request->GetRoot();

  // TODO check both project and camera intrinsics uuid are set

  ciFbCore->getData(*requestRoot, writer);

  return grpc::Status::OK;
}
grpc::Status FbCameraIntrinsicsService::TransferCameraIntrinsics(
    grpc::ServerContext* context, const flatbuffers::grpc::Message<seerep::fb::CameraIntrinsics>* request,
    flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response)
{
  (void)context;  // ignore this variable without causing warnings
  auto requestRoot = request->GetRoot();

  ciFbCore->setData(*requestRoot);

  return grpc::Status::OK;
}
}  // namespace seerep_server
