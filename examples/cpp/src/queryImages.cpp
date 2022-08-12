
#include <grpcpp/create_channel.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <iostream>

#include "seerep-com/image_service.grpc.fb.h"
#include "seerep-com/meta_operations.grpc.fb.h"
#include "seerep-com/point_service.grpc.fb.h"

std::shared_ptr<grpc::Channel> getChannel()
{
  return grpc::CreateChannel("localhost:9090", grpc::InsecureChannelCredentials());
}

std::optional<std::string> extractProject(flatbuffers::grpc::Message<seerep::fb::ProjectInfos>& response_msg,
                                          std::string projectName)
{
  const seerep::fb::ProjectInfos* response = response_msg.GetRoot();

  for (auto project : *response->projects())
  {
    if (project->name()->str() == projectName)
    {
      std::cout << project->name()->str() << " " << project->uuid()->str() << std::endl;
      return project->uuid()->str();
    }
  }
  return std::nullopt;
}

std::optional<std::string> getProjectUUID(std::string projectName, std::shared_ptr<grpc::Channel> channel)
{
  auto stub = seerep::fb::MetaOperations::NewStub(channel);

  flatbuffers::grpc::MessageBuilder mb;
  auto request_offset = seerep::fb::CreateEmpty(mb);
  mb.Finish(request_offset);
  auto request_msg = mb.ReleaseMessage<seerep::fb::Empty>();

  grpc::ClientContext context;
  flatbuffers::grpc::Message<seerep::fb::ProjectInfos> response_msg;
  auto status = stub->GetProjects(&context, request_msg, &response_msg);

  if (status.ok())
  {
    return extractProject(response_msg, projectName);
  }
  else
  {
    std::cerr << status.error_code() << ": " << status.error_message() << std::endl;
  }
  return std::nullopt;
}

void getImagesAndCreatePoints(std::shared_ptr<grpc::Channel> channel, std::string projectUUID)
{
  auto stub = seerep::fb::ImageService::NewStub(channel);

  flatbuffers::grpc::MessageBuilder mb;

  std::vector<flatbuffers::Offset<flatbuffers::String>> projectUUIDs;
  projectUUIDs.push_back(mb.CreateString(projectUUID));
  auto projectUUIDsOffset = mb.CreateVector(projectUUIDs);

  seerep::fb::QueryBuilder queryBuilder(mb);
  queryBuilder.add_projectuuid(projectUUIDsOffset);
  auto request = queryBuilder.Finish();
  mb.Finish(request);
  auto requestMsg = mb.ReleaseMessage<seerep::fb::Query>();

  grpc::ClientContext context;
  std::unique_ptr<grpc::ClientReader<flatbuffers::grpc::Message<seerep::fb::Image>>> reader(
      stub->GetImage(&context, requestMsg));

  std::vector<flatbuffers::grpc::Message<seerep::fb::PointStamped>> points;
  flatbuffers::grpc::Message<seerep::fb::Image> imageMsg;
  while (reader->Read(&imageMsg))
  {
    std::cout << imageMsg.GetRoot()->header()->uuid_msgs()->str() << std::endl;

    seerep::fb::PointStampedBuilder pointStampedBuilder(mb);
    // pointStampedBuilder.add_attribute();
    // pointStampedBuilder.add_header();
    // pointStampedBuilder.add_labels_general();
    // pointStampedBuilder.add_point();
    auto pointStamped = pointStampedBuilder.Finish();
    mb.Finish(pointStamped);
    points.push_back(mb.ReleaseMessage<seerep::fb::PointStamped>());
  }
}

int main()
{
  std::shared_ptr<grpc::Channel> channel = getChannel();

  auto projectUUID = getProjectUUID("testproject", channel);

  if (projectUUID)
  {
    std::cout << "Query Images" << std::endl;
    getImagesAndCreatePoints(channel, projectUUID.value());
  }
  std::cout << "Hello World" << std::endl;
  return 0;
}
