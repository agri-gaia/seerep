#ifndef SEEREP_SERVER_TF_SERVICE_H_
#define SEEREP_SERVER_TF_SERVICE_H_

// seerep
#include <seerep-com/tf-service.grpc.pb.h>
#include <seerep-core/seerep-core.h>
#include <seerep-pb-core/tf.h>

namespace seerep_server
{
class TfService final : public seerep::TfService::Service
{
public:
  TfService(std::shared_ptr<seerep_core::SeerepCore> seerepCore);

  grpc::Status TransferTransformStamped(grpc::ServerContext* context, const seerep::TransformStamped* transform,
                                        seerep::ServerResponse* response);
  grpc::Status GetFrames(grpc::ServerContext* context, const seerep::FrameQuery* frameQuery,
                         seerep::FrameInfos* response);
  grpc::Status GetTransformStamped(grpc::ServerContext* context, const seerep::TransformStampedQuery* transformQuery,
                                   seerep::TransformStamped* response);

private:
  std::shared_ptr<seerep_core_pb::TfPb> tfPb;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_TF_SERVICE_H_
