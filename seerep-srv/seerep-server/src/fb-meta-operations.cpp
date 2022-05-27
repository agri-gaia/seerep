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
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "create new project... ";
  const seerep::fb::ProjectCreation* requestMsg = request->GetRoot();
  seerep_core_msgs::ProjectInfo projectInfo;
  projectInfo.frameId = requestMsg->map_frame_id()->str();
  projectInfo.name = requestMsg->name()->str();
  projectInfo.uuid = boost::uuids::random_generator()();
  seerepCore->newProject(projectInfo);

  flatbuffers::grpc::MessageBuilder builder;
  auto nameOffset = builder.CreateString(projectInfo.name);
  auto uuidOffset = builder.CreateString(boost::lexical_cast<std::string>(projectInfo.uuid));
  auto responseOffset = seerep::fb::CreateProjectInfo(builder, nameOffset, uuidOffset);

  builder.Finish(responseOffset);
  *response = builder.ReleaseMessage<seerep::fb::ProjectInfo>();
  assert(response->Verify());

  return grpc::Status::OK;
}

grpc::Status FbMetaOperations::GetProjects(grpc::ServerContext* context,
                                           const flatbuffers::grpc::Message<seerep::fb::Empty>* request,
                                           flatbuffers::grpc::Message<seerep::fb::ProjectInfos>* response)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "query the project infos... ";
  auto projectInfos = seerepCore->getProjects();

  flatbuffers::grpc::MessageBuilder builder;
  std::vector<flatbuffers::Offset<seerep::fb::ProjectInfo>> projectInfosVector;
  for (auto projectInfo : projectInfos)
  {
    auto nameOffset = builder.CreateString(projectInfo.name);
    auto uuidOffset = builder.CreateString(boost::lexical_cast<std::string>(projectInfo.uuid));
    projectInfosVector.push_back(seerep::fb::CreateProjectInfo(builder, nameOffset, uuidOffset));
  }
  auto vectorOffset = builder.CreateVector(projectInfosVector);
  auto projectInfosOffset = seerep::fb::CreateProjectInfos(builder, vectorOffset);
  builder.Finish(projectInfosOffset);
  *response = builder.ReleaseMessage<seerep::fb::ProjectInfos>();
  assert(response->Verify());

  return grpc::Status::OK;
}

} /* namespace seerep_server */
