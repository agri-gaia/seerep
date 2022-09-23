#include "seerep-server/fb-point-cloud-service.h"

namespace seerep_server
{
FbPointCloudService::FbPointCloudService(std::shared_ptr<seerep_core::Core> seerepCore)
  : pointCloudFb(std::make_shared<seerep_core_fb::CoreFbPointCloud>(seerepCore))
{
}

grpc::Status
FbPointCloudService::GetPointCloud2(grpc::ServerContext* context,
                                    const flatbuffers::grpc::Message<seerep::fb::Query>* request,
                                    grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>* writer)
{
  // ignore without causing a warning
  (void)context;

  auto requestRoot = request->GetRoot();

  std::stringstream debuginfo;

  debuginfo << "sending point clouds with this query parameters: ";
  if (requestRoot->boundingbox() != NULL)
  {
    debuginfo << "\n bounding box min(x: " << requestRoot->boundingbox()->point_min()->x()
              << ", y: " << requestRoot->boundingbox()->point_min()->y()
              << ", z: " << requestRoot->boundingbox()->point_min()->z()
              << "), max(x: " << requestRoot->boundingbox()->point_max()->x()
              << ", y: " << requestRoot->boundingbox()->point_max()->y()
              << ", z: " << requestRoot->boundingbox()->point_max()->z() << ")";
  }
  if (requestRoot->timeinterval() != NULL)
  {
    debuginfo << "\n time interval (seconds since epoch: " << requestRoot->timeinterval()->time_min()->seconds()
              << ", nanoseconds since epoch:" << requestRoot->timeinterval()->time_max()->seconds() << ")";
  }
  if (requestRoot->label() != NULL)
  {
    debuginfo << "\n labels general:";
    for (auto label : *requestRoot->label())
    {
      debuginfo << " '" << label->str() << "' ";
    }
  }

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << debuginfo.rdbuf();

  try
  {
    pointCloudFb->getData(requestRoot, writer);
  }
  catch (std::runtime_error const& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  return grpc::Status::OK;
}

grpc::Status FbPointCloudService::TransferPointCloud2(
    grpc::ServerContext* context, grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>* reader,
    flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response)
{
  // ignore without causing warning
  (void)context;

  std::string answer = "Saved the flatbuffers point cloud message!";

  flatbuffers::grpc::Message<seerep::fb::PointCloud2> pointCloudMsg;

  while (reader->Read(&pointCloudMsg))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "received flatbuffers point cloud";

    auto pointCloud = pointCloudMsg.GetRoot();

    const std::string& uuidProject = pointCloud->header()->uuid_project()->str();

    if (!uuidProject.empty())
    {
      try
      {
        pointCloudFb->addData(*pointCloud);
      }
      catch (std::runtime_error const& e)
      {
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning) << e.what();

        seerep_server_util::createResponseFb(std::string(e.what()), seerep::fb::TRANSMISSION_STATE_FAILURE, response);

        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
      }
    }
    else
    {
      answer = "No project uuid in the header of the received flatbuffers point cloud message!";
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning) << answer;
    }
  }
  seerep_server_util::createResponseFb(answer, seerep::fb::TRANSMISSION_STATE_SUCCESS, response);

  return grpc::Status::OK;
}

} /* namespace seerep_server */
