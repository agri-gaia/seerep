#ifndef SEEREP_SERVER_PB_POINT_CLOUD_SERVICE_H_
#define SEEREP_SERVER_PB_POINT_CLOUD_SERVICE_H_

// seerep
#include <seerep-com/point-cloud-service.grpc.pb.h>
#include <seerep-core-pb/core-pb-pointcloud.h>
#include <seerep-core/core.h>

#include "util.hpp"

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_server
{
class PbPointCloudService final : public seerep::PointCloudService::Service
{
public:
  PbPointCloudService(std::shared_ptr<seerep_core::Core> seerepCore);

  grpc::Status GetPointCloud2(grpc::ServerContext* context, const seerep::Query* request,
                              grpc::ServerWriter<seerep::PointCloud2>* writer);

  grpc::Status TransferPointCloud2(grpc::ServerContext* context, const seerep::PointCloud2* pointCloud2,
                                   seerep::ServerResponse* response);

private:
  std::shared_ptr<seerep_core_pb::CorePbPointCloud> pointCloudPb;
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_PB_POINT_CLOUD_SERVICE_H_
