#ifndef SEEREP_SERVER_FB_IMAGE_SERVICE_H_
#define SEEREP_SERVER_FB_IMAGE_SERVICE_H_

// seerep
#include <seerep_com/image_service.grpc.fb.h>
#include <seerep_core/core.h>
#include <seerep_core_fb/core_fb_image.h>

#include "util.hpp"

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_server
{
class FbImageService final : public seerep::fb::ImageService::Service
{
public:
  FbImageService(std::shared_ptr<seerep_core::Core> seerepCore);

  grpc::Status GetImage(grpc::ServerContext* context, const flatbuffers::grpc::Message<seerep::fb::Query>* request,
                        grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::Image>>* writer) override;
  grpc::Status TransferImage(grpc::ServerContext* context,
                             grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::Image>>* reader,
                             flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response) override;
  grpc::Status AddBoundingBoxes2dLabeled(
      grpc::ServerContext* context,
      grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::BoundingBoxes2DLabeledStamped>>* reader,
      flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response) override;

private:
  std::shared_ptr<seerep_core_fb::CoreFbImage> imageFb;
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_FB_IMAGE_SERVICE_H_
