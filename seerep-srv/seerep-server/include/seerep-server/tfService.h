#ifndef SEEREP_SERVER_TF_SERVICE_H_
#define SEEREP_SERVER_TF_SERVICE_H_

// seerep
#include <seerep-com/tfService.grpc.pb.h>
#include <seerep-core/project-overview.h>

namespace seerep_server
{
class TfService final : public seerep::TfService::Service
{
public:
  TfService(std::shared_ptr<seerep_core::ProjectOverview> projectOverview);

  grpc::Status TransferTransformStamped(grpc::ServerContext* context, const seerep::TransformStamped* transform,
                                        seerep::ServerResponse* response);
  grpc::Status GetFrames(grpc::ServerContext* context, const seerep::FrameQuery* frameQuery,
                         seerep::FrameInfos* response);
  grpc::Status GetTransformStamped(grpc::ServerContext* context, const seerep::TransformStampedQuery* transformQuery,
                                   seerep::TransformStamped* response);

private:
  std::shared_ptr<seerep_core::ProjectOverview> projectOverview;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_TF_SERVICE_H_
