#include "seerep_server/fb_point_service.h"

namespace seerep_server
{
FbPointService::FbPointService(std::shared_ptr<seerep_core::Core> seerepCore)
  : pointFb(std::make_shared<seerep_core_fb::CoreFbPoint>(seerepCore))
{
}

grpc::Status FbPointService::GetPoint(
    grpc::ServerContext* context,
    const flatbuffers::grpc::Message<seerep::fb::Query>* request,
    grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::PointStamped>>*
        writer)
{
  (void)context;  // ignore that variable without causing warnings
  auto requestRoot = request->GetRoot();

  std::stringstream debuginfo;
  debuginfo << "sending images with this query parameters:";
  if (requestRoot->polygon() != NULL)
  {
    for (auto point : *requestRoot->polygon()->vertices())
    {
      debuginfo << "bounding box vertex (" << point->x() << ", " << point->y()
                << ") \n";
    }
    debuginfo << "bounding box z " << requestRoot->polygon()->z() << " /";
    debuginfo << "\n bounding box height " << requestRoot->polygon()->height()
              << " /";
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

  if (requestRoot->polygon() != NULL)
  {
    for (auto point : *(requestRoot->polygon()->vertices()))
    {
      debuginfo << "bounding box vertex (" << point->x() << ", " << point->y()
                << ") /";
    }
    debuginfo << "bounding box z " << requestRoot->polygon()->z() << " /";
    debuginfo << "bounding box height " << requestRoot->polygon()->height()
              << " /";
  }
  if (requestRoot->timeinterval() != NULL)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "in time interval ("
        << requestRoot->timeinterval()->time_min()->seconds() << "/"
        << requestRoot->timeinterval()->time_max()->seconds() << ")";
  }
  try
  {
    pointFb->getData(requestRoot, writer);
  }
  catch (std::runtime_error const& e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from
    // string to uuid also catching core doesn't have project with uuid error
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
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

grpc::Status FbPointService::TransferPoint(
    grpc::ServerContext* context,
    grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::PointStamped>>*
        reader,
    flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response)
{
  (void)context;  // ignore that variable without causing warnings
  std::string answer = "everything stored!";

  flatbuffers::grpc::Message<seerep::fb::PointStamped> pointMsg;
  std::vector<std::pair<std::string, boost::uuids::uuid>> projectPointUuids;

  // write the point data to hdf5
  while (reader->Read(&pointMsg))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "received point... ";
    auto point = pointMsg.GetRoot();

    std::string uuidProject = point->header()->uuid_project()->str();
    if (!uuidProject.empty())
    {
      try
      {
        projectPointUuids.push_back(
            { uuidProject, pointFb->addDataToHdf5(point) });
      }
      catch (std::runtime_error const& e)
      {
        // mainly catching "invalid uuid string" when transforming uuid_project
        // from string to uuid also catching core doesn't have project with uuid
        // error
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
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
        std::string msg =
            "Unknown failure occurred. Possible memory corruption";
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
            << msg;
        seerep_server_util::createResponseFb(
            msg, seerep::fb::TRANSMISSION_STATE_FAILURE, response);
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
      }
    }
    else
    {
      answer = "a msg had no project uuid!";
    }
  }

  // build the indices from the written data
  try
  {
    pointFb->buildIndices(projectPointUuids);
  }
  catch (std::runtime_error const& e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project
    // from string to uuid also catching core doesn't have project with uuid
    // error
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
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

grpc::Status FbPointService::AddAttribute(
    grpc::ServerContext* context,
    grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::AttributesStamped>>*
        reader,
    flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response)
{
  (void)context;  // ignore that variable without causing warnings
  std::string answer = "everything stored!";

  flatbuffers::grpc::Message<seerep::fb::AttributesStamped> attributesStampedMsg;
  while (reader->Read(&attributesStampedMsg))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "received image... ";
    auto attributesStamped = attributesStampedMsg.GetRoot();

    std::string uuidProject =
        attributesStamped->header()->uuid_project()->str();
    if (!uuidProject.empty())
    {
      try
      {
        pointFb->addAttributes(*attributesStamped);
      }
      catch (std::runtime_error const& e)
      {
        // mainly catching "invalid uuid string" when transforming uuid_project
        // from string to uuid also catching core doesn't have project with uuid
        // error
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
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
        std::string msg =
            "Unknown failure occurred. Possible memory corruption";
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
            << msg;
        seerep_server_util::createResponseFb(
            msg, seerep::fb::TRANSMISSION_STATE_FAILURE, response);
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
      }
    }
    else
    {
      answer = "a msg had no project uuid!";
    }
  }

  seerep_server_util::createResponseFb(
      answer, seerep::fb::TRANSMISSION_STATE_SUCCESS, response);

  return grpc::Status::OK;
}

} /* namespace seerep_server */
