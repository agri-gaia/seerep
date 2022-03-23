#include "seerep-server/fb-meta-operations.h"

namespace seerep_server
{
FbMetaOperations::FbMetaOperations(std::shared_ptr<seerep_core::Core> seerepCore) : seerepCore(seerepCore)
{
}

grpc::Status FbMetaOperations::CreateProject(grpc::ServerContext* context,
                                             const flatbuffers::grpc::Message<seerep::fb::ProjectCreation>* request,
                                             flatbuffers::grpc::Message<seerep::fb::ProjectInfo>* response)
{
  std::cout << "create new project... " << std::endl;
  const seerep::fb::ProjectCreation* requestMsg = request->GetRoot();
  seerep_core_msgs::ProjectInfo projectInfo;
  projectInfo.frameId = requestMsg->map_frame_id()->str();
  projectInfo.name = requestMsg->name()->str();
  projectInfo.uuid = boost::uuids::random_generator()();
  seerepCore->newProject(projectInfo);

  flatbuffers::grpc::MessageBuilder builder;
  auto responseOffset =
      seerep::fb::CreateProjectInfo(builder, builder.CreateString(projectInfo.name),
                                    builder.CreateString(boost::lexical_cast<std::string>(projectInfo.uuid)));

  builder.Finish(responseOffset);
  *response = builder.ReleaseMessage<seerep::fb::ProjectInfo>();
  assert(response->Verify());

  return grpc::Status::OK;
}

grpc::Status FbMetaOperations::GetProjects(grpc::ServerContext* context,
                                           const flatbuffers::grpc::Message<seerep::fb::Empty>* request,
                                           flatbuffers::grpc::Message<seerep::fb::ProjectInfos>* response)
{
  std::cout << "query the project infos... " << std::endl;
  auto projectInfos = seerepCore->getProjects();

  flatbuffers::grpc::MessageBuilder builder;
  seerep::fb::ProjectInfosBuilder projectInfosBuilder(builder);
  seerep::fb::ProjectInfoBuilder projectInfoBuilder(builder);

  for (auto projectInfo : projectInfos)
  {
    projectInfoBuilder.add_name(builder.CreateString(projectInfo.name));
    projectInfoBuilder.add_uuid(builder.CreateString(boost::lexical_cast<std::string>(projectInfo.uuid)));
    projectInfosBuilder.add_projects(projectInfoBuilder.Finish());
  }

  builder.Finish(projectInfosBuilder.Finish());
  *response = builder.ReleaseMessage<seerep::fb::ProjectInfos>();
  assert(response->Verify());

  return grpc::Status::OK;
}

} /* namespace seerep_server */
