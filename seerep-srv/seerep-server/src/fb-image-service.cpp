#include "seerep-server/fb-image-service.h"

namespace seerep_server
{
FbImageService::FbImageService(std::shared_ptr<seerep_core::Core> seerepCore)
  : imageFb(std::make_shared<seerep_core_fb::CoreFbImage>(seerepCore))
{
}

grpc::Status FbImageService::GetImage(grpc::ServerContext* context,
                                      const flatbuffers::grpc::Message<seerep::fb::Query>* request,
                                      grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::Image>>* writer)
{
  auto requestRoot = request->GetRoot();
  std::cout << "sending images in bounding box min(" << requestRoot->boundingbox()->point_min()->x() << "/"
            << requestRoot->boundingbox()->point_min()->y() << "/" << requestRoot->boundingbox()->point_min()->z()
            << "), max(" << requestRoot->boundingbox()->point_max()->x() << "/"
            << requestRoot->boundingbox()->point_max()->y() << "/" << requestRoot->boundingbox()->point_max()->z()
            << ")"
            << " and time interval (" << requestRoot->timeinterval()->time_min()->seconds() << "/"
            << requestRoot->timeinterval()->time_max()->seconds() << ")" << std::endl;

  // std::vector<flatbuffers::Offset<seerep::fb::Image>> images;
  try
  {
    // images =
    imageFb->getData(*requestRoot, writer);
  }
  catch (std::runtime_error e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
    // also catching core doesn't have project with uuid error
    std::cout << e.what() << std::endl;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }

  // if (!images.empty())
  // {
  //   std::cout << "Found " << images.size() << " images that match the query" << std::endl;
  //   for (const auto& img : images)
  //   {
  //     writer->Write(img);
  //   }
  // }
  // else
  // {
  //   std::cout << "Found NOTHING that matches the query" << std::endl;
  // }
  return grpc::Status::OK;
}

grpc::Status FbImageService::TransferImage(grpc::ServerContext* context,
                                           grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::Image>>* reader,
                                           flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response)
{
  //   std::cout << "received image... " << std::endl;

  //   if (!image->header().uuid_project().empty())
  //   {
  //     boost::uuids::uuid uuid;
  //     try
  //     {
  //       boost::uuids::string_generator gen;
  //       uuid = gen(image->header().uuid_project());

  //       boost::uuids::uuid uuidImg = imagePb->addData(*image);

  //       response->set_message(boost::lexical_cast<std::string>(uuidImg));
  //       response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  //       return grpc::Status::OK;
  //     }
  //     catch (std::runtime_error e)
  //     {
  //       // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
  //       // also catching core doesn't have project with uuid error
  //       std::cout << e.what() << std::endl;
  //       response->set_message(std::string(e.what()));
  //       response->set_transmission_state(seerep::ServerResponse::FAILURE);
  //       return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  //     }
  //   }
  //   else
  //   {
  //     std::cout << "project_uuid is empty!" << std::endl;
  //     response->set_message("project_uuid is empty!");
  //     response->set_transmission_state(seerep::ServerResponse::FAILURE);
  //     return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "project_uuid is empty!");
  //   }
}

} /* namespace seerep_server */
