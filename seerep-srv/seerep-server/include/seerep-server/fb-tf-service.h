#ifndef SEEREP_SERVER_FB_TF_SERVICE_H_
#define SEEREP_SERVER_FB_TF_SERVICE_H_

// seerep
#include <seerep-com/tf-service.grpc.fb.h>
#include <seerep-core/core.h>
#include <seerep-core-fb/core-fb-tf.h>

namespace seerep_server
{
class FbTfService final : public seerep::fb::TfService::Service
{
public:
  FbTfService(std::shared_ptr<seerep_core::Core> seerepCore);

  grpc::Status
  TransferTransformStamped(grpc::ServerContext* context,
                           grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::TransformStamped>>* reader,
                           flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response);
  grpc::Status GetFrames(grpc::ServerContext* context,
                         const flatbuffers::grpc::Message<seerep::fb::FrameQuery>* request,
                         flatbuffers::grpc::Message<seerep::fb::FrameInfos>* response);
  grpc::Status GetTransformStamped(grpc::ServerContext* context,
                                   const flatbuffers::grpc::Message<seerep::fb::TransformStampedQuery>* request,
                                   flatbuffers::grpc::Message<seerep::fb::TransformStamped>* response);

private:
  std::shared_ptr<seerep_core_fb::CoreFbTf> tfFb;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_FB_TF_SERVICE_H_
