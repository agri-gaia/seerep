#include "seerep-server/point-cloud-service.h"

namespace seerep_server
{
PointCloudService::PointCloudService(std::shared_ptr<seerep_core::Core> seerepCore)
  : pointCloudPb(std::make_shared<seerep_core_pb::CorePbPointCloud>(seerepCore))
{
}

grpc::Status PointCloudService::GetPointCloud2(grpc::ServerContext* context, const seerep::Query* request,
                                               grpc::ServerWriter<seerep::PointCloud2>* writer)
{
  std::cout << "sending point cloud in bounding box min(" << request->boundingbox().point_min().x() << "/"
            << request->boundingbox().point_min().y() << "/" << request->boundingbox().point_min().z() << "), max("
            << request->boundingbox().point_max().x() << "/" << request->boundingbox().point_max().y() << "/"
            << request->boundingbox().point_max().z() << ")"
            << " and time interval (" << request->timeinterval().time_min() << "/" << request->timeinterval().time_max()
            << ")" << std::endl;

  std::vector<seerep::PointCloud2> pointClouds;
  try
  {
    pointClouds = pointCloudPb->getData(*request);
  }
  catch (std::runtime_error e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
    // also catching core doesn't have project with uuid error
    std::cout << e.what() << std::endl;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }

  if (!pointClouds.empty())
  {
    std::cout << "Found " << pointClouds.size() << " pointclouds that match the query" << std::endl;
    for (const seerep::PointCloud2& pc : pointClouds)
    {
      writer->Write(pc);
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

      boost::uuids::uuid uuidImg = pointCloudPb->addData(*pointCloud2);

      response->set_message(boost::lexical_cast<std::string>(uuidImg));
      response->set_transmission_state(seerep::ServerResponse::SUCCESS);
      return grpc::Status::OK;
    }
    catch (std::runtime_error e)
    {
      // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
      // also catching core doesn't have project with uuid error
      std::cout << e.what() << std::endl;
      response->set_message(std::string(e.what()));
      response->set_transmission_state(seerep::ServerResponse::FAILURE);
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
    }
  }
  else
  {
    std::cout << "project_uuid is empty!" << std::endl;
    response->set_message("project_uuid is empty!");
    response->set_transmission_state(seerep::ServerResponse::FAILURE);
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "project_uuid is empty!");
  }
}

} /* namespace seerep_server */
