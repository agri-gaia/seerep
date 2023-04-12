#include "seerep-server/pb-point-cloud-service.h"

namespace seerep_server
{
PbPointCloudService::PbPointCloudService(std::shared_ptr<seerep_core::Core> seerepCore)
  : pointCloudPb(std::make_shared<seerep_core_pb::CorePbPointCloud>(seerepCore))
{
}

grpc::Status PbPointCloudService::GetPointCloud2(grpc::ServerContext* context, const seerep::pb::Query* request,
                                                 grpc::ServerWriter<seerep::pb::PointCloud2>* writer)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
      << "sending point cloud in bounding box center(" << request->boundingboxstamped().boundingbox().center_point().x()
      << "/" << request->boundingboxstamped().boundingbox().center_point().y() << "/"
      << request->boundingboxstamped().boundingbox().center_point().z() << "), spatial extent("
      << request->boundingboxstamped().boundingbox().spatial_extent().x() << "/"
      << request->boundingboxstamped().boundingbox().spatial_extent().y() << "/"
      << request->boundingboxstamped().boundingbox().spatial_extent().z() << ")"
      << " and time interval (" << request->timeinterval().time_min().seconds() << "/"
      << request->timeinterval().time_max().seconds() << ")";

  std::vector<seerep::pb::PointCloud2> pointClouds;
  try
  {
    pointClouds = pointCloudPb->getData(*request);

    if (!pointClouds.empty())
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
          << "Found " << pointClouds.size() << " pointclouds that match the query";
      for (const seerep::pb::PointCloud2& pc : pointClouds)
      {
        writer->Write(pc);
      }
    }
    else
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "Found NOTHING that matches the query";
    }
  }
  catch (std::runtime_error const& e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
    // also catching core doesn't have project with uuid error
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (const std::exception& e)
  {
    // specific handling for all exceptions extending std::exception, except
    // std::runtime_error which is handled explicitly
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (...)
  {
    // catch any other errors (that we have no information about)
    std::string msg = "Unknown failure occurred. Possible memory corruption";
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
  }
  return grpc::Status::OK;
}

grpc::Status PbPointCloudService::TransferPointCloud2(grpc::ServerContext* context,
                                                      const seerep::pb::PointCloud2* pointCloud2,
                                                      seerep::pb::ServerResponse* response)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "received point clouds... ";

  if (!pointCloud2->header().uuid_project().empty())
  {
    try
    {
      boost::uuids::uuid pointCloudUuid = pointCloudPb->addData(*pointCloud2);

      seerep_server_util::createResponsePb(boost::lexical_cast<std::string>(pointCloudUuid),
                                           seerep::pb::ServerResponse::SUCCESS, response);

      return grpc::Status::OK;
    }
    catch (std::runtime_error const& e)
    {
      // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
      // also catching core doesn't have project with uuid error
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();

      seerep_server_util::createResponsePb(std::string(e.what()), seerep::pb::ServerResponse::FAILURE, response);

      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
    }
    catch (const std::exception& e)
    {
      // specific handling for all exceptions extending std::exception, except
      // std::runtime_error which is handled explicitly
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
      seerep_server_util::createResponsePb(std::string(e.what()), seerep::pb::ServerResponse::FAILURE, response);
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
    }
    catch (...)
    {
      // catch any other errors (that we have no information about)
      std::string msg = "Unknown failure occurred. Possible memory corruption";
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
      seerep_server_util::createResponsePb(msg, seerep::pb::ServerResponse::FAILURE, response);
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
    }
  }
  else
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning) << "project_uuid is empty!";
    seerep_server_util::createResponsePb("project_uuid is empty!", seerep::pb::ServerResponse::FAILURE, response);

    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "project_uuid is empty!");
  }
}

} /* namespace seerep_server */
