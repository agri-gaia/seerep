#include "seerep-server/query-data.h"

namespace seerep_server
{
QueryData::QueryData(std::shared_ptr<seerep_core::ProjectOverview> projectOverview) : projectOverview(projectOverview)
{
}

grpc::Status QueryData::GetImage(grpc::ServerContext* context, const seerep::Query* request,
                                 grpc::ServerWriter<seerep::Image>* writer)
{
  std::cout << "sending images in bounding box min(" << request->boundingbox().point_min().x() << "/"
            << request->boundingbox().point_min().y() << "/" << request->boundingbox().point_min().z() << "), max("
            << request->boundingbox().point_max().x() << "/" << request->boundingbox().point_max().y() << "/"
            << request->boundingbox().point_max().z() << ")"
            << " and time interval (" << request->timeinterval().time_min() << "/" << request->timeinterval().time_max()
            << ")" << std::endl;

  std::vector<std::vector<std::optional<seerep::Image>>> images = projectOverview->getImage(*request);
  if (!images.empty())
  {
    std::cout << "Found images in " << images.size() << " projects that match the query" << std::endl;

    for (const std::vector<std::optional<seerep::Image>>& resultPerProject : images)
    {
      std::cout << "Found " << resultPerProject.size() << " images in this projects that match the query" << std::endl;
      for (const std::optional<seerep::Image>& img : resultPerProject)
      {
        writer->Write(img.value());
      }
    }
  }
  else
  {
    std::cout << "Found NOTHING that matches the query" << std::endl;
  }
  return grpc::Status::OK;
}

grpc::Status QueryData::GetPointCloud2(grpc::ServerContext* context, const seerep::Query* request,
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

} /* namespace seerep_server */
