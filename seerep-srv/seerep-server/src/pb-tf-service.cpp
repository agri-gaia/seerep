#include "seerep-server/pb-tf-service.h"

namespace seerep_server
{
PbTfService::PbTfService(std::shared_ptr<seerep_core::Core> seerepCore)
  : tfPb(std::make_shared<seerep_core_pb::CorePbTf>(seerepCore))
{
}

grpc::Status PbTfService::TransferTransformStamped(grpc::ServerContext* context,
                                                   const seerep::TransformStamped* transform,
                                                   seerep::ServerResponse* response)
{
  std::cout << "received transform... " << std::endl;

  if (!transform->header().uuid_project().empty())
  {
    boost::uuids::uuid uuid;
    try
    {
      boost::uuids::string_generator gen;
      uuid = gen(transform->header().uuid_project());

      tfPb->addData(*transform);

      response->set_message("added transform");
      response->set_transmission_state(seerep::ServerResponse::SUCCESS);
      return grpc::Status::OK;
    }
    catch (std::runtime_error e)
    {
      // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
      // also catching core doesn't have project with uuid error
      std::cout << e.what() << std::endl;
      response->set_message(std::string(e.what()));
      response->set_transmission_state(seerep::ServerResponse::FAILURE);
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
    }
  }
  else
  {
    std::cout << "project_uuid is empty!" << std::endl;
    response->set_message("project_uuid is empty!");
    response->set_transmission_state(seerep::ServerResponse::FAILURE);
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "project_uuid is empty!");
  }
}

grpc::Status PbTfService::GetFrames(grpc::ServerContext* context, const seerep::FrameQuery* frameQuery,
                                    seerep::FrameInfos* response)
{
  boost::uuids::uuid uuid;
  try
  {
    boost::uuids::string_generator gen;
    uuid = gen(frameQuery->projectuuid());

    for (auto framename : tfPb->getFrames(uuid))
    {
      response->add_frames(framename);
    }
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

grpc::Status PbTfService::GetTransformStamped(grpc::ServerContext* context,
                                              const seerep::TransformStampedQuery* transformQuery,
                                              seerep::TransformStamped* response)
{
  try
  {
    auto result = tfPb->getData(*transformQuery);
    if (result)
    {
      *response = result.value();
    }
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
