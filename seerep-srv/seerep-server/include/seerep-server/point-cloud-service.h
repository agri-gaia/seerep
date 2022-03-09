#ifndef SEEREP_SERVER_POINT_CLOUD_SERVICE_H_
#define SEEREP_SERVER_POINT_CLOUD_SERVICE_H_

// seerep
#include <seerep-com/point-cloud-service.grpc.pb.h>
#include <seerep-core/seerep-core.h>

namespace seerep_server
{
class PointCloudService final : public seerep::PointCloudService::Service
{
public:
  PointCloudService(std::shared_ptr<seerep_core::ProjectOverview> projectOverview);

  grpc::Status GetPointCloud2(grpc::ServerContext* context, const seerep::Query* request,
                              grpc::ServerWriter<seerep::PointCloud2>* writer);

  grpc::Status TransferPointCloud2(grpc::ServerContext* context, const seerep::PointCloud2* pointCloud2,
                                   seerep::ServerResponse* response);

private:
  std::shared_ptr<seerep_core::SeerepCore> seerepCore;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_POINT_CLOUD_SERVICE_H_
