#include "seerep-server/pb-image-service.h"

namespace seerep_server
{
PbImageService::PbImageService(std::shared_ptr<seerep_core::Core> seerepCore)
  : imagePb(std::make_shared<seerep_core_pb::CorePbImage>(seerepCore))
{
}

grpc::Status PbImageService::GetImage(grpc::ServerContext* context, const seerep::Query* request,
                                      grpc::ServerWriter<seerep::Image>* writer)
{
  (void)context;  // ignore that variable without causing warnings
  std::cout << "sending images in bounding box min(" << request->boundingbox().point_min().x() << "/"
            << request->boundingbox().point_min().y() << "/" << request->boundingbox().point_min().z() << "), max("
            << request->boundingbox().point_max().x() << "/" << request->boundingbox().point_max().y() << "/"
            << request->boundingbox().point_max().z() << ")"
            << " and time interval (" << request->timeinterval().time_min().seconds() << "/"
            << request->timeinterval().time_max().seconds() << ")" << std::endl;

  std::vector<seerep::Image> images;
  try
  {
    images = imagePb->getData(*request);
  }
  catch (std::runtime_error const& e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
    // also catching core doesn't have project with uuid error
    std::cout << e.what() << std::endl;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }

  if (!images.empty())
  {
    std::cout << "Found " << images.size() << " images that match the query" << std::endl;
    for (const seerep::Image& img : images)
    {
      writer->Write(img);
    }
  }
  else
  {
    std::cout << "Found NOTHING that matches the query" << std::endl;
  }
  return grpc::Status::OK;
}

grpc::Status PbImageService::TransferImage(grpc::ServerContext* context, const seerep::Image* image,
                                           seerep::ServerResponse* response)
{
  (void)context;  // ignore that variable without causing warnings
  std::cout << "received image... " << std::endl;

  if (!image->header().uuid_project().empty())
  {
    boost::uuids::uuid uuid;
    try
    {
      boost::uuids::string_generator gen;
      uuid = gen(image->header().uuid_project());

      boost::uuids::uuid uuidImg = imagePb->addData(*image);

      seerep_server_util::createResponsePb(boost::lexical_cast<std::string>(uuidImg), seerep::ServerResponse::SUCCESS,
                                           response);

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

} /* namespace seerep_server */
