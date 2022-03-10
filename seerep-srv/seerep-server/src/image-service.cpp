#include "seerep-server/image-service.h"

namespace seerep_server
{
ImageService::ImageService(std::shared_ptr<seerep_core::Core> seerepCore)
  : imagePb(std::make_shared<seerep_core_pb::CorePbImage>(seerepCore))
{
}

grpc::Status ImageService::GetImage(grpc::ServerContext* context, const seerep::Query* request,
                                    grpc::ServerWriter<seerep::Image>* writer)
{
  std::cout << "sending images in bounding box min(" << request->boundingbox().point_min().x() << "/"
            << request->boundingbox().point_min().y() << "/" << request->boundingbox().point_min().z() << "), max("
            << request->boundingbox().point_max().x() << "/" << request->boundingbox().point_max().y() << "/"
            << request->boundingbox().point_max().z() << ")"
            << " and time interval (" << request->timeinterval().time_min() << "/" << request->timeinterval().time_max()
            << ")" << std::endl;

  std::vector<seerep::Image> images = imagePb->getData(*request);
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

grpc::Status ImageService::TransferImage(grpc::ServerContext* context, const seerep::Image* image,
                                         seerep::ServerResponse* response)
{
  std::cout << "received image... " << std::endl;

  if (!image->header().uuid_project().empty())
  {
    boost::uuids::uuid uuid;
    try
    {
      boost::uuids::string_generator gen;
      uuid = gen(image->header().uuid_project());
    }
    catch (std::runtime_error e)
    {
      // mainly catching "invalid uuid string"
      std::cout << e.what() << std::endl;
      return grpc::Status::CANCELLED;
    }
    boost::uuids::uuid uuidImg = imagePb->addData(*image);
    response->set_message(boost::lexical_cast<std::string>(uuidImg));
    response->set_transmission_state(seerep::ServerResponse::SUCCESS);
    return grpc::Status::OK;
  }
  else
  {
    std::cout << "project_uuid is empty!" << std::endl;
    return grpc::Status::CANCELLED;
  }
}

} /* namespace seerep_server */
