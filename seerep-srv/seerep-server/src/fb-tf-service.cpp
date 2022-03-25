#include "seerep-server/fb-tf-service.h"

namespace seerep_server
{
FbTfService::FbTfService(std::shared_ptr<seerep_core::Core> seerepCore)
  : tfFb(std::make_shared<seerep_core_fb::CoreFbTf>(seerepCore))
{
}

grpc::Status FbTfService::TransferTransformStamped(
    grpc::ServerContext* context, grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::TransformStamped>>* reader,
    flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response)
{
  std::cout << "received transform... " << std::endl;
  std::string answer = "everything stored!";

  flatbuffers::grpc::Message<seerep::fb::TransformStamped> tfMsg;
  while (reader->Read(&tfMsg))
  {
    auto transform = tfMsg.GetRoot();
    std::string uuidProject = transform->header()->uuid_project()->str();
    if (!uuidProject.empty())
    {
      try
      {
        tfFb->addData(*transform);
      }
      catch (std::runtime_error e)
      {
        // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
        // also catching core doesn't have project with uuid error
        std::cout << e.what() << std::endl;

        flatbuffers::grpc::MessageBuilder builder;
        auto msg = builder.CreateString(std::string(e.what()));
        seerep::fb::ServerResponseBuilder responseBuilder(builder);
        responseBuilder.add_message(msg);
        responseBuilder.add_transmission_state(seerep::fb::TRANSMISSION_STATE_FAILURE);
        auto responseOffset = responseBuilder.Finish();
        builder.Finish(responseOffset);
        *response = builder.ReleaseMessage<seerep::fb::ServerResponse>();
        assert(response->Verify());

        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
      }
    }
    else
    {
      answer = "a msg had no project uuid!";
    }
  }
  flatbuffers::grpc::MessageBuilder builder;
  auto msg = builder.CreateString(answer);
  seerep::fb::ServerResponseBuilder responseBuilder(builder);
  responseBuilder.add_message(msg);
  responseBuilder.add_transmission_state(seerep::fb::TRANSMISSION_STATE_SUCCESS);
  auto responseOffset = responseBuilder.Finish();
  builder.Finish(responseOffset);
  *response = builder.ReleaseMessage<seerep::fb::ServerResponse>();
  assert(response->Verify());

  return grpc::Status::OK;
}

grpc::Status FbTfService::GetFrames(grpc::ServerContext* context,
                                    const flatbuffers::grpc::Message<seerep::fb::FrameQuery>* request,
                                    flatbuffers::grpc::Message<seerep::fb::FrameInfos>* response)
{
  boost::uuids::uuid uuid;
  try
  {
    boost::uuids::string_generator gen;
    uuid = gen(request->GetRoot()->projectuuid()->str());

    auto frames = tfFb->getFrames(uuid);

    flatbuffers::grpc::MessageBuilder builder;

    std::vector<flatbuffers::Offset<flatbuffers::String>> framesOffset;
    for (auto framename : tfFb->getFrames(uuid))
    {
      framesOffset.push_back(builder.CreateString(framename));
    }

    seerep::fb::FrameInfosBuilder frameinfosbuilder(builder);
    frameinfosbuilder.add_frames(builder.CreateVector(framesOffset));
    auto frameinfosOffset = frameinfosbuilder.Finish();
    *response = builder.ReleaseMessage<seerep::fb::FrameInfos>();
    assert(response->Verify());
  }
  catch (std::runtime_error e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
    // also catching core doesn't have project with uuid error
    std::cout << e.what() << std::endl;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  return grpc::Status::OK;
}

grpc::Status
FbTfService::GetTransformStamped(grpc::ServerContext* context,
                                 const flatbuffers::grpc::Message<seerep::fb::TransformStampedQuery>* request,
                                 flatbuffers::grpc::Message<seerep::fb::TransformStamped>* response)
{
  try
  {
    tfFb->getData(*request->GetRoot(), response);
  }
  catch (std::runtime_error e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
    // also catching core doesn't have project with uuid error
    std::cout << e.what() << std::endl;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }

  return grpc::Status::OK;
}
} /* namespace seerep_server */
