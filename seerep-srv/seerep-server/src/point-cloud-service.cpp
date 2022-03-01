#include "seerep-server/point-cloud-service.h"

namespace seerep_server
{
PointCloudService::PointCloudService(std::shared_ptr<seerep_core::ProjectOverview> projectOverview)
  : projectOverview(projectOverview)
{
}

grpc::Status PointCloudService::GetPointCloud2(grpc::ServerContext* context, const seerep::Query* request,
                                               grpc::ServerWriter<seerep::PointCloud2>* writer)
{
  std::cout << "sending point cloud in bounding box min(" << request->boundingbox().point_min().x() << "/"
            << request->boundingbox().point_min().y() << "/" << request->boundingbox().point_min().z() << "), max("
            << request->boundingbox().point_max().x() << "/" << request->boundingbox().point_max().y() << "/"
            << request->boundingbox().point_max().z() << ")" << std::endl;

  std::vector<std::vector<std::optional<seerep::PointCloud2>>> pointclouds = projectOverview->getPointCloud(*request);
  if (!pointclouds.empty())
  {
    std::cout << "Found pointclouds in " << pointclouds.size() << " projects that match the query" << std::endl;

    for (const std::vector<std::optional<seerep::PointCloud2>>& resultPerProject : pointclouds)
    {
      std::cout << "Found " << resultPerProject.size() << " pointclouds in this projects that match the query"
                << std::endl;
      for (const std::optional<seerep::PointCloud2>& pc : resultPerProject)
      {
        writer->Write(pc.value());
      }
    }
  }
  else
  {
    std::cout << "Found NOTHING that matches the query" << std::endl;
  }
  return grpc::Status::OK;
}

grpc::Status PointCloudService::TransferPointCloud2(grpc::ServerContext* context,
                                                    const seerep::PointCloud2* pointCloud2,
                                                    seerep::ServerResponse* response)
{
  std::cout << "received point clouds... " << std::endl;

  if (!pointCloud2->header().uuid_project().empty())
  {
    boost::uuids::uuid uuid;
    try
    {
      boost::uuids::string_generator gen;
      uuid = gen(pointCloud2->header().uuid_project());
    }
    catch (std::runtime_error e)
    {
      // mainly catching "invalid uuid string"
      std::cout << e.what() << std::endl;
      return grpc::Status::CANCELLED;
    }

    projectOverview->addPointCloud(*pointCloud2, uuid);
    response->set_message("okidoki");
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
