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
  (void)context;  // ignore that variable without causing warnings
  std::cout << "received transform... " << std::endl;

  if (!transform->header().uuid_project().empty())
  {
    boost::uuids::uuid uuid;
    try
    {
      boost::uuids::string_generator gen;
      uuid = gen(transform->header().uuid_project());

      tfPb->addData(*transform);

      seerep_server_util::createResponsePb("added transform", seerep::ServerResponse::SUCCESS, response);

      return grpc::Status::OK;
    }
    catch (std::runtime_error const& e)
    {
      // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
      // also catching core doesn't have project with uuid error
      std::cout << e.what() << std::endl;
      seerep_server_util::createResponsePb(std::string(e.what()), seerep::ServerResponse::FAILURE, response);

      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
    }
  }
  else
  {
    std::cout << "project_uuid is empty!" << std::endl;
    seerep_server_util::createResponsePb("project_uuid is empty!", seerep::ServerResponse::FAILURE, response);

    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "project_uuid is empty!");
  }
}

grpc::Status PbTfService::GetFrames(grpc::ServerContext* context, const seerep::FrameQuery* frameQuery,
                                    seerep::FrameInfos* response)
{
  (void)context;  // ignore that variable without causing warnings
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
  catch (std::runtime_error const& e)
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
  (void)context;  // ignore that variable without causing warnings
  try
  {
    auto result = tfPb->getData(*transformQuery);
    if (result)
    {
      *response = result.value();
    }
  }
  catch (std::runtime_error const& e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
    // also catching core doesn't have project with uuid error
    std::cout << e.what() << std::endl;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }

  return grpc::Status::OK;
}
} /* namespace seerep_server */
