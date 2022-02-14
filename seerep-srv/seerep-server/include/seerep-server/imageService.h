#ifndef SEEREP_SERVER_IMAGE_SERVICE_H_
#define SEEREP_SERVER_IMAGE_SERVICE_H_

// seerep
#include <seerep-com/imageService.grpc.pb.h>
#include <seerep-core/project-overview.h>

namespace seerep_server
{
class ImageService final : public seerep::ImageService::Service
{
public:
  ImageService(std::shared_ptr<seerep_core::ProjectOverview> projectOverview);

  grpc::Status GetImage(grpc::ServerContext* context, const seerep::Query* request,
                        grpc::ServerWriter<seerep::Image>* writer);

  grpc::Status TransferImage(grpc::ServerContext* context, const seerep::Image* image, seerep::ServerResponse* response);

private:
  std::shared_ptr<seerep_core::ProjectOverview> projectOverview;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_IMAGE_SERVICE_H_
