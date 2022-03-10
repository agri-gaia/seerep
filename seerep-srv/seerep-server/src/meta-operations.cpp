#include "seerep-server/meta-operations.h"

namespace seerep_server
{
MetaOperations::MetaOperations(std::shared_ptr<seerep_core::Core> seerepCore) : seerepCore(seerepCore)
{
}

grpc::Status MetaOperations::CreateProject(grpc::ServerContext* context, const seerep::ProjectCreation* request,
                                           seerep::ProjectInfo* response)
{
  std::cout << "create new project... " << std::endl;
  seerep_core_msgs::ProjectInfo projectInfo;
  projectInfo.frameId = request->mapframeid();
  projectInfo.name = request->name();
  projectInfo.uuid = boost::uuids::random_generator()();
  seerepCore->newProject(projectInfo);

  response->set_name(projectInfo.name);
  response->set_uuid(boost::lexical_cast<std::string>(projectInfo.uuid));

  return grpc::Status::OK;
}

grpc::Status MetaOperations::GetProjects(grpc::ServerContext* context, const google::protobuf::Empty* request,
                                         seerep::ProjectInfos* response)
{
  std::cout << "query the project infos... " << std::endl;
  auto projectInfos = seerepCore->getProjects();

  for (auto projectInfo : projectInfos)
  {
    auto responseProjectInfo = response->add_projects();
    responseProjectInfo->set_name(projectInfo.name);
    responseProjectInfo->set_uuid(boost::lexical_cast<std::string>(projectInfo.uuid));
  }

  return grpc::Status::OK;
}

} /* namespace seerep_server */
