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

  // check if the distortion model is found in kCameraDistortionModels
  bool distortion_model_err =
      !requestRoot->distortion_model() ||
      std::find(std::begin(seerep_server_constants::kCameraDistortionModels),
                std::end(seerep_server_constants::kCameraDistortionModels),
                requestRoot->distortion_model()->c_str()) == std::end(seerep_server_constants::kCameraDistortionModels);

  bool distortion_mat_err = !requestRoot->distortion();
  bool intrinsics_mat_err = !requestRoot->intrinsic_matrix() || requestRoot->intrinsic_matrix()->size() != 9 ||
                            (*requestRoot->intrinsic_matrix())[0] == 0 || (*requestRoot->intrinsic_matrix())[4] == 0;

  bool rectification_mat_err =
      !requestRoot->rectification_matrix() ||
      (requestRoot->rectification_matrix()->size() > 0 && requestRoot->rectification_matrix()->size() != 9);

  bool projection_mat_err = !requestRoot->projection_matrix() || (requestRoot->projection_matrix()->size() > 0 &&
                                                                  requestRoot->projection_matrix()->size() != 12);

  if (distortion_model_err || distortion_mat_err || intrinsics_mat_err || rectification_mat_err || projection_mat_err)
  {
    response_message = "";
    if (distortion_model_err)
    {
      response_message += "The distortion model is not set or invalid. ";
    }
    if (distortion_mat_err)
    {
      response_message += "The distortion matrix is not set. ";
    }
    if (intrinsics_mat_err)
    {
      response_message += "Instrinsics matrix is not set, dimensions are incorrect or the focal length values are "
                          "invalid. ";
    }
    if (rectification_mat_err)
    {
      response_message += "Rectification matrix is not set or dimensions are incorrect. ";
    }
    if (projection_mat_err)
    {
      response_message += "Projection matrix is not set or dimensions are incorrect.";
    }
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << response_message;
    seerep_server_util::createResponseFb(response_message, seerep::fb::TRANSMISSION_STATE_FAILURE, response);
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, response_message);
  }

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
