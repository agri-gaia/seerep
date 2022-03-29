#ifndef SEEREP_SERVER_FB_IMAGE_SERVICE_H_
#define SEEREP_SERVER_FB_IMAGE_SERVICE_H_

// seerep
#include <seerep-com/image-service.grpc.fb.h>
#include <seerep-core-fb/core-fb-image.h>
#include <seerep-core/core.h>

namespace seerep_server
{
class FbImageService final : public seerep::fb::ImageService::Service
{
public:
  FbImageService(std::shared_ptr<seerep_core::Core> seerepCore);

  grpc::Status GetImage(grpc::ServerContext* context, const flatbuffers::grpc::Message<seerep::fb::Query>* request,
                        grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::Image>>* writer);
  grpc::Status TransferImage(grpc::ServerContext* context,
                             grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::Image>>* reader,
                             flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response);
  grpc::Status AddBoundingBoxes2dLabeled(
      grpc::ServerContext* context,
      grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::BoundingBoxes2DLabeledStamped>>* reader,
      flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response);

private:
  std::shared_ptr<seerep_core_fb::CoreFbImage> imageFb;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_FB_IMAGE_SERVICE_H_
