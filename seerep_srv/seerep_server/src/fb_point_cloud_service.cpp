#include "seerep_server/fb_point_cloud_service.h"

namespace seerep_server
{
FbPointCloudService::FbPointCloudService(
    std::shared_ptr<seerep_core::Core> seerepCore)
  : pointCloudFb(std::make_shared<seerep_core_fb::CoreFbPointCloud>(seerepCore))
{
}

grpc::Status FbPointCloudService::GetPointCloud2(
    grpc::ServerContext* context,
    const flatbuffers::grpc::Message<seerep::fb::Query>* request,
    grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>*
        writer)
{
  // ignore without causing a warning
  (void)context;

  auto requestRoot = request->GetRoot();

  std::stringstream debuginfo;

  debuginfo << "sending point clouds with this query parameters: ";
  if (requestRoot->polygon() != NULL)
  {
    for (auto&& point : *requestRoot->polygon()->vertices())
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
    for (auto&& point : *(requestRoot->polygonSensorPosition()->vertices()))
    {
      debuginfo << "bounding box vertex (" << point->x() << ", " << point->y()
                << ") /";
    }
    debuginfo << "bounding box z " << requestRoot->polygonSensorPosition()->z()
              << " /";
    debuginfo << "bounding box height "
              << requestRoot->polygonSensorPosition()->height() << " /";
  }
  if (requestRoot->timeintervals() != NULL)
  {
    for (auto&& timeinterval : *requestRoot->timeintervals())
    {
      debuginfo << "\n time interval (" << timeinterval->time_min()->seconds()
                << "/" << timeinterval->time_max()->seconds() << ")";
    }
  }
  if (requestRoot->label() != NULL)
  {
    debuginfo << "\n labels general";
    for (auto&& labelCategory : *requestRoot->label())
    {
      debuginfo << "category: " << labelCategory->category()->c_str() << "; ";
      for (auto&& label : *labelCategory->labels())
      {
        debuginfo << "'" << label->label()->str() << "' ";
      }
    }
  }

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
      << debuginfo.rdbuf();

  try
  {
    pointCloudFb->getData(requestRoot, writer);
  }
  catch (std::runtime_error const& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (const std::exception& e)
  {
    // specific handling for all exceptions extending std::exception, except
    // std::runtime_error which is handled explicitly
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
        << e.what();
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

grpc::Status FbPointCloudService::TransferPointCloud2(
    grpc::ServerContext* context,
    grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>*
        reader,
    flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response)
{
  // ignore without causing warning
  (void)context;

  std::string answer = "Saved the flatbuffers point cloud message!";

  flatbuffers::grpc::Message<seerep::fb::PointCloud2> pointCloudMsg;
  std::vector<std::pair<std::string, boost::uuids::uuid>> projectPclUuids;

  // write pointcloud data to hdf5
  while (reader->Read(&pointCloudMsg))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "received flatbuffers point cloud";

    try
    {
      auto pointCloud = pointCloudMsg.GetRoot();

      const std::string& uuidProject =
          pointCloud->header()->uuid_project()->str();
      if (!uuidProject.empty())
      {
        projectPclUuids.push_back(
            { uuidProject, pointCloudFb->addDataToHdf5(*pointCloud) });
      }
      else
      {
        answer = "No project uuid in the header of the received flatbuffers "
                 "point cloud message!";
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
            << answer;
      }
    }
    catch (std::runtime_error const& e)
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
          << e.what();

      seerep_server_util::createResponseFb(
          std::string(e.what()), seerep::fb::TRANSMISSION_STATE_FAILURE,
          response);

      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
    }
    catch (const std::exception& e)
    {
      // specific handling for all exceptions extending std::exception, except
      // std::runtime_error which is handled explicitly
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
          << e.what();
      seerep_server_util::createResponseFb(
          std::string(e.what()), seerep::fb::TRANSMISSION_STATE_FAILURE,
          response);
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
    }
    catch (...)
    {
      // catch any other errors (that we have no information about)
      std::string msg = "Unknown failure occurred. Possible memory corruption";
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
          << msg;
      seerep_server_util::createResponseFb(
          msg, seerep::fb::TRANSMISSION_STATE_FAILURE, response);
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
    }
  }

  // build the indices from the written data
  try
  {
    pointCloudFb->buildIndices(projectPclUuids);
  }
  catch (std::runtime_error const& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << e.what();

    seerep_server_util::createResponseFb(std::string(e.what()),
                                         seerep::fb::TRANSMISSION_STATE_FAILURE,
                                         response);

    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (const std::exception& e)
  {
    // specific handling for all exceptions extending std::exception, except
    // std::runtime_error which is handled explicitly
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
        << e.what();
    seerep_server_util::createResponseFb(std::string(e.what()),
                                         seerep::fb::TRANSMISSION_STATE_FAILURE,
                                         response);
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (...)
  {
    // catch any other errors (that we have no information about)
    std::string msg = "Unknown failure occurred. Possible memory corruption";
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
    seerep_server_util::createResponseFb(
        msg, seerep::fb::TRANSMISSION_STATE_FAILURE, response);
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
  }

  seerep_server_util::createResponseFb(
      answer, seerep::fb::TRANSMISSION_STATE_SUCCESS, response);

  return grpc::Status::OK;
}

grpc::Status FbPointCloudService::AddLabels(
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
        << "received label... ";
    auto label = labelMsg.GetRoot();

    if (label->projectUuid()->str().empty())
    {
      answer = "a msg had no project uuid!";
    }
    else
    {
      if (!label->labels())
      {
        answer = "a msg had no labels!";
      }
      else
      {
        try
        {
          pointCloudFb->addLabel(*label);
        }
        catch (std::runtime_error const& e)
        {
          // mainly catching "invalid uuid string" when transforming uuid_project
          // from string to uuid also catching core doesn't have project with uuid error
          BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
              << e.what();

          seerep_server_util::createResponseFb(
              std::string(e.what()), seerep::fb::TRANSMISSION_STATE_FAILURE,
              response);

          return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
        }
        catch (const std::exception& e)
        {
          // specific handling for all exceptions extending std::exception,
          // except std::runtime_error which is handled explicitly
          BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
              << e.what();
          seerep_server_util::createResponseFb(
              std::string(e.what()), seerep::fb::TRANSMISSION_STATE_FAILURE,
              response);
          return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
        }
        catch (...)
        {
          // catch any other errors (that we have no information about)
          std::string msg =
              "Unknown failure occurred. Possible memory corruption";
          BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
              << msg;
          seerep_server_util::createResponseFb(
              msg, seerep::fb::TRANSMISSION_STATE_FAILURE, response);
          return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
        }
      }
    }
  }
  seerep_server_util::createResponseFb(
      answer, seerep::fb::TRANSMISSION_STATE_SUCCESS, response);

  return grpc::Status::OK;
}

} /* namespace seerep_server */
