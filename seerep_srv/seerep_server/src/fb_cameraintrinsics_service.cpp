#include "seerep_server/fb_cameraintrinsics_service.h"

namespace seerep_server
{
FbCameraIntrinsicsService::FbCameraIntrinsicsService(std::shared_ptr<seerep_core::Core> seerepCore)
  : ciFbCore(std::make_shared<seerep_core_fb::CoreFbCameraIntrinsics>(seerepCore))
{
}
grpc::Status FbCameraIntrinsicsService::GetCameraIntrinsics(
    grpc::ServerContext* context, const flatbuffers::grpc::Message<seerep::fb::CameraIntrinsicsQuery>* request,
    flatbuffers::grpc::Message<seerep::fb::CameraIntrinsics>* response)
{
  (void)context;  // ignore this variable without causing warnings
  auto requestRoot = request->GetRoot();

  std::stringstream debuginfo;
  debuginfo << "sending images with this query parameters:";

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << debuginfo.rdbuf();

  if (requestRoot->uuid_camera_intrinsics() != NULL)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "fetching camera intrinsics against camera intrinsics uuid " << requestRoot->uuid_camera_intrinsics()->str();
  }
  if (requestRoot->uuid_project() != NULL)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "fetching camera intrinsics against project uuid " << requestRoot->uuid_project()->str();
  }

  try
  {
    ciFbCore->getData(*requestRoot, response);
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
grpc::Status FbCameraIntrinsicsService::TransferCameraIntrinsics(
    grpc::ServerContext* context, const flatbuffers::grpc::Message<seerep::fb::CameraIntrinsics>* request,
    flatbuffers::grpc::Message<seerep::fb::ServerResponse>* response)
{
  (void)context;  // ignore this variable without causing warnings
  auto requestRoot = request->GetRoot();

  std::string response_message = "Camera Intrinsics saved";

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "received camera instrinsics for storage ";

  if (requestRoot->header()->uuid_msgs()->str().empty())
  {
    response_message = "No UUID for Camera Intrinsics set";
  }
  else if (requestRoot->header()->uuid_project()->str().empty())
  {
    response_message = "No UUID for Project set";
  }
  else if (requestRoot->maximum_viewing_distance() == 0)
  {
    std::string msg = "Max Viewing Distance is not set. Using default value of 0.";
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;

    response_message = "Max Viewing Distance is the distance to the farthest object in view and is used to compute a "
                       "frustrum of the camera's view. This value is not set. Using default value of 0.";
  }
  else
  {
    try
    {
      ciFbCore->setData(*requestRoot);
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
  seerep_server_util::createResponseFb(response_message, seerep::fb::TRANSMISSION_STATE_SUCCESS, response);

  return grpc::Status::OK;
}
}  // namespace seerep_server
