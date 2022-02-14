#include "seerep-server/meta-operations.h"

namespace seerep_server
{
MetaOperations::MetaOperations(std::shared_ptr<seerep_core::ProjectOverview> projectOverview)
  : projectOverview(projectOverview)
{
}

grpc::Status MetaOperations::CreateProject(grpc::ServerContext* context, const seerep::ProjectCreation* request,
                                           seerep::ProjectInfo* response)
{
  std::cout << "create new project... " << std::endl;
  projectOverview->newProject(request->name(), request->mapframeid(), response);

  return grpc::Status::OK;
}

grpc::Status MetaOperations::GetProjects(grpc::ServerContext* context, const google::protobuf::Empty* request,
                                         seerep::ProjectInfos* response)
{
  std::cout << "query the project infos... " << std::endl;
  projectOverview->getProjects(response);

  return grpc::Status::OK;
}

} /* namespace seerep_server */
