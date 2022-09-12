#ifndef SEEREP_SERVER_FB_POINT_CLOUD_SERVICE_H_
#define SEEREP_SERVER_FB_POINT_CLOUD_SERVICE_H_

// seerep
#include <seerep-com/point_cloud_service.grpc.fb.h>
#include <seerep-core-fb/core-fb-pointcloud.h>
#include <seerep-core/core.h>

#include "util.hpp"

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_server
{
class FbPointCloudService final : public seerep::fb::PointCloudService::Service
{
public:
  FbPointCloudService(std::shared_ptr<seerep_core::Core> seerepCore);
  grpc::Status GetPointCloud2(grpc::ServerContext* context,
                              const flatbuffers::grpc::Message<seerep::fb::Query>* request,
                              grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>* writer) override;
  grpc::Status TransferPointCloud2(grpc::ServerContext* context,
                                   grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>* reader,
                                   flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response) override;
  void createResponse(std::string msg, seerep::fb::TRANSMISSION_STATE state,
                      flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response);

  std::shared_ptr<seerep_core_fb::CoreFbPointCloud> pointCloudFb;
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};
} /* namespace seerep_server */
#endif  // SEEREP_SERVER_FB_POINT_CLOUD_SERVICE_H_
