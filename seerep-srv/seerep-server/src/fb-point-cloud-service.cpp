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
}

grpc::Status FbPointCloudService::TransferPointCloud2(
    grpc::ServerContext* context, grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>* reader,
    flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response)
{
  (void)context;  // ignore
  std::string answer = "everything stored!";

  flatbuffers::grpc::Message<seerep::fb::PointCloud2> pointCloudMsg;
  while (reader->Read(&pointCloudMsg))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "received point cloud ...";
    auto pointCloud = pointCloudMsg.GetRoot();

    std::string uuidProject = pointCloud->header()->uuid_project()->str();
    if (!uuidProject.empty())
    {
      try
      {
        pointCloudFb->addData(*pointCloud);
      }
      catch (std::runtime_error const& e)
      {
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();

        seerep_server_util::createResponseFb(std::string(e.what()), seerep::fb::TRANSMISSION_STATE_FAILURE, response);

        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
      }
    }
    else
    {
      answer = "a msg had no project uuid!";
    }
  }
  seerep_server_util::createResponseFb(answer, seerep::fb::TRANSMISSION_STATE_SUCCESS, response);

  return grpc::Status::OK;
}

} /* namespace seerep_server */
