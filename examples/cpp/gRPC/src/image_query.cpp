
#include "../include/image_query.h"

namespace seerep_cpp_examples
{

ImageQuery::ImageQuery(std::shared_ptr<grpc::Channel> channel)
  : channel_(channel), fbb_(flatbuffers::grpc::MessageBuilder())
{
}

std::optional<std::string>
ImageQuery::getProjectUUID(const std::string& projectName)
{
  auto metaStub = seerep::fb::MetaOperations::NewStub(channel_);
  auto requestOffset = seerep::fb::CreateEmpty(fbb_);
  fbb_.Finish(requestOffset);
  auto requestMsg = fbb_.ReleaseMessage<seerep::fb::Empty>();

  grpc::ClientContext context;
  flatbuffers::grpc::Message<seerep::fb::ProjectInfos> responseMsg;
  auto status = metaStub->GetProjects(&context, requestMsg, &responseMsg);

  if (status.ok())
  {
    auto response = responseMsg.GetRoot();
    for (const auto& project : *response->projects())
    {
      if (project->name()->str() == projectName)
      {
        return project->uuid()->str();
      }
    }
  }
  else
  {
    throw std::runtime_error(status.error_message());
  }
  return std::nullopt;
}

flatbuffers::grpc::Message<seerep::fb::Query>
ImageQuery::createQuery(const std::string& projectUUID)
{
  std::vector<flatbuffers::Offset<flatbuffers::String>> projectUUIDs;
  projectUUIDs.push_back(fbb_.CreateString(projectUUID));
  auto projectUuidVec = fbb_.CreateVector(projectUUIDs);

  seerep::fb::QueryBuilder queryBuilder(fbb_);
  queryBuilder.add_projectuuid(projectUuidVec);
  /* add extra query parameters here */
  auto requestOffset = queryBuilder.Finish();
  fbb_.Finish(requestOffset);
  return fbb_.ReleaseMessage<seerep::fb::Query>();
}

void ImageQuery::getImages(const std::string& projectName)
{
  auto imageStub = seerep::fb::ImageService::NewStub(channel_);
  std::optional<std::string> projectUUID = getProjectUUID(projectName);
  if (projectUUID.has_value())
  {
    flatbuffers::grpc::Message<seerep::fb::Image> responseMsg;
    grpc::ClientContext context;
    auto stream =
        imageStub->GetImage(&context, createQuery(projectUUID.value()));
    while (stream->Read(&responseMsg))
    {
      auto response = responseMsg.GetRoot();
      /* do something usefull with the images matching the query */
      std::cout << "Received image with UUID: "
                << response->header()->uuid_msgs()->str() << std::endl;
    }
  }
  else
  {
    throw std::runtime_error("Project not found");
  }
}

}  // namespace seerep_cpp_examples

int main()
{
  std::shared_ptr<grpc::Channel> grpcChannel =
      grpc::CreateChannel("localhost:9090", grpc::InsecureChannelCredentials());
  seerep_cpp_examples::ImageQuery imageQuery(grpcChannel);
  imageQuery.getImages("testproject");
}
