
#include "../include/tf_query.h"

namespace seerep_cpp_examples
{

TfQuery::TfQuery(std::shared_ptr<grpc::Channel> channel)
  : channel_(channel), fbb_(flatbuffers::grpc::MessageBuilder())
{
}

std::optional<std::string>
TfQuery::getProjectUUID(const std::string& projectName)
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

flatbuffers::grpc::Message<seerep::fb::TransformStampedQuery>
TfQuery::createQuery(const std::string& projectUUID,
                     const std::string& childFrame,
                     const std::string& parentFrame, const int& seconds,
                     const int& nanos)
{
  auto projectUUIDOffst = fbb_.CreateString(projectUUID);
  auto childFrameOffset = fbb_.CreateString(childFrame);
  auto parentFrameOffset = fbb_.CreateString(parentFrame);

  seerep::fb::TimestampBuilder timestampBuilder(fbb_);
  timestampBuilder.add_seconds(seconds);
  timestampBuilder.add_nanos(nanos);
  auto timestampOffset = timestampBuilder.Finish();

  seerep::fb::HeaderBuilder headerBuilder(fbb_);
  headerBuilder.add_frame_id(parentFrameOffset);
  headerBuilder.add_stamp(timestampOffset);
  headerBuilder.add_uuid_project(projectUUIDOffst);
  auto headerOffset = headerBuilder.Finish();

  seerep::fb::TransformStampedQueryBuilder queryBuilder(fbb_);
  queryBuilder.add_header(headerOffset);
  queryBuilder.add_child_frame_id(childFrameOffset);
  /* add extra query parameters here */
  auto requestOffset = queryBuilder.Finish();
  fbb_.Finish(requestOffset);
  return fbb_.ReleaseMessage<seerep::fb::TransformStampedQuery>();
}

void TfQuery::getTf(const std::string& projectName,
                    const std::string& childFrame,
                    const std::string& parentFrame, const int& seconds,
                    const int& nanos)
{
  auto tfStub = seerep::fb::TfService::NewStub(channel_);
  std::optional<std::string> projectUUID = getProjectUUID(projectName);
  if (projectUUID.has_value())
  {
    grpc::ClientContext context;
    flatbuffers::grpc::Message<seerep::fb::TransformStamped> responseMsg;
    tfStub->GetTransformStamped(&context,
                                createQuery(projectUUID.value(), childFrame,
                                            parentFrame, seconds, nanos),
                                &responseMsg);

    auto response = responseMsg.GetRoot();
    /* do something usefull with the images matching the query */
    std::cout << "Received tf with childframe: "
              << response->child_frame_id()->str() << std::endl;
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
  seerep_cpp_examples::TfQuery TfQuery(grpcChannel);
  TfQuery.getTf("testproject", "map", "camera", 1661336508, 0);
}
