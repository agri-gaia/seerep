#ifndef SEEREP_SERVER_META_OPERATIONS_H_
#define SEEREP_SERVER_META_OPERATIONS_H_

// seerep
#include <seerep-com/meta-operations.grpc.pb.h>
#include <seerep-core/project-overview.h>

namespace seerep_server
{
class MetaOperations final : public seerep::MetaOperations::Service
{
public:
  MetaOperations(std::shared_ptr<seerep_core::ProjectOverview> projectOverview);

  grpc::Status CreateProject(grpc::ServerContext* context, const seerep::ProjectCreation* request,
                             seerep::ProjectInfo* response);
  grpc::Status GetProjects(grpc::ServerContext* context, const google::protobuf::Empty* request,
                           seerep::ProjectInfos* response);

private:
  std::shared_ptr<seerep_core::ProjectOverview> projectOverview;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_META_OPERATIONS_H_
