#include "seerep_server/fb_image_service.h"

namespace seerep_server
{
FbImageService::FbImageService(std::shared_ptr<seerep_core::Core> seerepCore)
  : imageFb(std::make_shared<seerep_core_fb::CoreFbImage>(seerepCore))
{
}

grpc::Status FbImageService::GetImage(
    grpc::ServerContext* context,
    const flatbuffers::grpc::Message<seerep::fb::Query>* request,
    grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::Image>>* writer)
{
  (void)context;
  auto requestRoot = request->GetRoot();

  std::stringstream debuginfo;
  debuginfo << "sending images with this query parameters:";
  if (requestRoot->polygon() != NULL)
  {
    for (auto point : *requestRoot->polygon()->vertices())
    {
      debuginfo << "bounding box vertex (" << point->x() << ", " << point->y()
                << ") /";
    }
    debuginfo << "bounding box z " << requestRoot->polygon()->z() << " /";
    debuginfo << "bounding box height " << requestRoot->polygon()->height()
              << " /";
  }
  if (requestRoot->polygonSensorPosition() != NULL)
  {
    for (auto point : *(requestRoot->polygonSensorPosition()->vertices()))
    {
      debuginfo << "bounding box vertex (" << point->x() << ", " << point->y()
                << ") /";
    }
    debuginfo << "bounding box z " << requestRoot->polygonSensorPosition()->z()
              << " /";
    debuginfo << "bounding box height "
              << requestRoot->polygonSensorPosition()->height() << " /";
  }
  if (requestRoot->timeinterval() != NULL)
  {
    debuginfo << "\n time interval ("
              << requestRoot->timeinterval()->time_min()->seconds() << "/"
              << requestRoot->timeinterval()->time_max()->seconds() << ")";
  }
  if (requestRoot->label() != NULL)
  {
    debuginfo << "\n labels general";
    for (auto labelCategory : *requestRoot->label())
    {
      debuginfo << "category: " << labelCategory->category()->c_str() << "; ";
      for (auto label : *labelCategory->labels())
      {
        debuginfo << "'" << label->label()->str() << "' ";
      }
    }
  }

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
      << debuginfo.rdbuf();

  try
  {
    imageFb->getData(requestRoot, writer);
  }
  catch (const std::exception& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
        << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (...)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
        << unkownErrorMsg;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, unkownErrorMsg);
  }

  return grpc::Status::OK;
}

grpc::Status FbImageService::TransferImage(
    grpc::ServerContext* context,
    grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::Image>>* reader,
    flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response)
{
  (void)context;
  std::string answer = "Everything stored!";

  flatbuffers::grpc::Message<seerep::fb::Image> imageMsg;
  std::vector<std::pair<std::string, boost::uuids::uuid>> projectImgUuids;

  while (reader->Read(&imageMsg))
  {
    const seerep::fb::Image* image = imageMsg.GetRoot();

    if ((image->uri() == nullptr || image->uri()->str().empty()) &&
        image->data() == nullptr)
    {
      answer = "Image message must contain an uri or payload";
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, answer);
    }

    const std::string projectUuid = image->header()->uuid_project()->str();

    if (!projectUuid.empty())
    {
      try
      {
        projectImgUuids.push_back(
            { projectUuid, imageFb->addDataToHdf5(*image) });
      }
      catch (const std::exception& e)
      {
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
            << e.what();
        seerep_server_util::createResponseFb(
            std::string(e.what()), seerep::fb::TRANSMISSION_STATE_FAILURE,
            response);
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
      }
      catch (...)
      {
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
            << unkownErrorMsg;
        seerep_server_util::createResponseFb(
            unkownErrorMsg, seerep::fb::TRANSMISSION_STATE_FAILURE, response);
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, unkownErrorMsg);
      }
    }
    else
    {
      answer = "An received image had no project UUID!";
    }
  }

  // create the indices when all image data was written to hdf5
  try
  {
    imageFb->buildIndices(projectImgUuids);
  }
  catch (const std::exception& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
        << e.what();
    seerep_server_util::createResponseFb(std::string(e.what()),
                                         seerep::fb::TRANSMISSION_STATE_FAILURE,
                                         response);
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (...)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
        << unkownErrorMsg;
    seerep_server_util::createResponseFb(
        unkownErrorMsg, seerep::fb::TRANSMISSION_STATE_FAILURE, response);
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, unkownErrorMsg);
  }

  seerep_server_util::createResponseFb(
      answer, seerep::fb::TRANSMISSION_STATE_SUCCESS, response);

  return grpc::Status::OK;
}

grpc::Status FbImageService::AddLabels(
    grpc::ServerContext* context,
    grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::DatasetUuidLabel>>*
        reader,
    flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response)
{
  (void)context;  // ignore that variable without causing warnings
  std::string answer = "everything stored!";

  flatbuffers::grpc::Message<seerep::fb::DatasetUuidLabel> labelMsg;
  while (reader->Read(&labelMsg))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "received label...";
    auto label = labelMsg.GetRoot();

    if (label->projectUuid()->str().empty())
    {
      answer = "a msg had no project uuid!";
    }
    else
    {
      if (!label->labels())
      {
        answer = "a msg had no label!";
      }
      else
      {
        try
        {
          imageFb->addLabel(*label);
        }
        catch (const std::exception& e)
        {
          BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
              << e.what();
          seerep_server_util::createResponseFb(
              std::string(e.what()), seerep::fb::TRANSMISSION_STATE_FAILURE,
              response);
          return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
        }
        catch (...)
        {
          BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
              << unkownErrorMsg;
          seerep_server_util::createResponseFb(
              unkownErrorMsg, seerep::fb::TRANSMISSION_STATE_FAILURE, response);
          return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                              unkownErrorMsg);
        }
      }
    }
  }
  seerep_server_util::createResponseFb(
      answer, seerep::fb::TRANSMISSION_STATE_SUCCESS, response);

  return grpc::Status::OK;
}
} /* namespace seerep_server */
