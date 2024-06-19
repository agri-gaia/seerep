#include "seerep_server/fb_image_service.h"

namespace seerep_server
{
FbImageService::FbImageService(std::shared_ptr<seerep_core::Core> seerepCore)
  : imageFb(std::make_shared<seerep_core_fb::CoreFbImage>(seerepCore))
{
}

grpc::Status FbImageService::GetImage(grpc::ServerContext* context,
                                      const flatbuffers::grpc::Message<seerep::fb::Query>* request,
                                      grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::Image>>* writer)
{
  (void)context;  // ignore that variable without causing warnings
  auto requestRoot = request->GetRoot();

  std::stringstream debuginfo;
  debuginfo << "sending images with this query parameters:";
  if (requestRoot->polygon() != NULL)
  {
    for (auto point : *requestRoot->polygon()->vertices())
    {
      debuginfo << "bounding box vertex (" << point->x() << ", " << point->y() << ") /";
    }
    debuginfo << "bounding box z " << requestRoot->polygon()->z() << " /";
    debuginfo << "bounding box height " << requestRoot->polygon()->height() << " /";
  }
  if (requestRoot->timeinterval() != NULL)
  {
    debuginfo << "\n time interval (" << requestRoot->timeinterval()->time_min()->seconds() << "/"
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

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << debuginfo.rdbuf();

  if (requestRoot->polygon() != NULL)
  {
    for (auto point : *(requestRoot->polygon()->vertices()))
    {
      debuginfo << "bounding box vertex (" << point->x() << ", " << point->y() << ") /";
    }
    debuginfo << "bounding box z " << requestRoot->polygon()->z() << " /";
    debuginfo << "bounding box height " << requestRoot->polygon()->height() << " /";
  }
  if (requestRoot->timeinterval() != NULL)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "in time interval (" << requestRoot->timeinterval()->time_min()->seconds() << "/"
        << requestRoot->timeinterval()->time_max()->seconds() << ")";
  }
  try
  {
    imageFb->getData(requestRoot, writer);
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

grpc::Status FbImageService::TransferImage(grpc::ServerContext* context,
                                           grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::Image>>* reader,
                                           flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response)
{
  (void)context;  // ignore that variable without causing warnings
  std::string answer = "everything stored!";

  flatbuffers::grpc::Message<seerep::fb::Image> imageMsg;
  std::unordered_map<std::string, std::vector<boost::uuids::uuid>> projectsImgUuids;
  while (reader->Read(&imageMsg))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "received image... ";
    auto image = imageMsg.GetRoot();
    std::string uuidProject = image->header()->uuid_project()->str();
    if (!uuidProject.empty())
    {
      try
      {
        if (projectsImgUuids.find(uuidProject) == projectsImgUuids.end())
        {
          projectsImgUuids.insert({ uuidProject, std::vector<boost::uuids::uuid>{} });
        }
        projectsImgUuids.at(uuidProject).push_back(imageFb->addDataToHdf5(*image));
      }
      catch (std::runtime_error const& e)
      {
        // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
        // also catching core doesn't have project with uuid error
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();

        seerep_server_util::createResponseFb(std::string(e.what()), seerep::fb::TRANSMISSION_STATE_FAILURE, response);

        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
      }
      catch (const std::exception& e)
      {
        // specific handling for all exceptions extending std::exception, except
        // std::runtime_error which is handled explicitly
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
        seerep_server_util::createResponseFb(std::string(e.what()), seerep::fb::TRANSMISSION_STATE_FAILURE, response);
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
      }
      catch (...)
      {
        // catch any other errors (that we have no information about)
        std::string msg = "Unknown failure occurred. Possible memory corruption";
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
        seerep_server_util::createResponseFb(msg, seerep::fb::TRANSMISSION_STATE_FAILURE, response);
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
      }
    }
    else
    {
      answer = "a msg had no project uuid!";
    }
  }

  // defer index creation
  imageFb->buildIndices(projectsImgUuids);

  seerep_server_util::createResponseFb(answer, seerep::fb::TRANSMISSION_STATE_SUCCESS, response);

  return grpc::Status::OK;
}

grpc::Status FbImageService::AddBoundingBoxes2dLabeled(
    grpc::ServerContext* context,
    grpc::ServerReader<flatbuffers::grpc::Message<seerep::fb::BoundingBoxes2DLabeledStamped>>* reader,
    flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response)
{
  (void)context;  // ignore that variable without causing warnings
  std::string answer = "everything stored!";

  flatbuffers::grpc::Message<seerep::fb::BoundingBoxes2DLabeledStamped> bbsMsg;
  while (reader->Read(&bbsMsg))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "received BoundingBoxes2DLabeledStamped... ";
    auto bbslabeled = bbsMsg.GetRoot();

    std::string uuidProject = bbslabeled->header()->uuid_project()->str();
    if (uuidProject.empty())
    {
      answer = "a msg had no project uuid!";
    }
    else
    {
      if (!bbslabeled->labels_bb())
      {
        answer = "a msg had no bounding boxes!";
      }
      else
      {
        try
        {
          imageFb->addBoundingBoxesLabeled(*bbslabeled);
        }
        catch (std::runtime_error const& e)
        {
          // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
          // also catching core doesn't have project with uuid error
          BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();

          seerep_server_util::createResponseFb(std::string(e.what()), seerep::fb::TRANSMISSION_STATE_FAILURE, response);

          return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
        }
        catch (const std::exception& e)
        {
          // specific handling for all exceptions extending std::exception, except
          // std::runtime_error which is handled explicitly
          BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
          seerep_server_util::createResponseFb(std::string(e.what()), seerep::fb::TRANSMISSION_STATE_FAILURE, response);
          return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
        }
        catch (...)
        {
          // catch any other errors (that we have no information about)
          std::string msg = "Unknown failure occurred. Possible memory corruption";
          BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
          seerep_server_util::createResponseFb(msg, seerep::fb::TRANSMISSION_STATE_FAILURE, response);
          return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
        }
      }
    }
  }
  seerep_server_util::createResponseFb(answer, seerep::fb::TRANSMISSION_STATE_SUCCESS, response);

  return grpc::Status::OK;
}
} /* namespace seerep_server */
