#include "seerep_server/pb_camera_intrinsics_service.h"

namespace seerep_server
{
PbCameraIntrinsicsService::PbCameraIntrinsicsService(std::shared_ptr<seerep_core::Core> seerepCore)
  : camIntrinsicsPb(std::make_shared<seerep_core_pb::CorePbCameraIntrinsics>(seerepCore))
{
}

grpc::Status PbCameraIntrinsicsService::TransferCameraIntrinsics(grpc::ServerContext* context,
                                                                 const seerep::pb::CameraIntrinsics* camintrinsics,
                                                                 seerep::pb::ServerResponse* response)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "received camera intrinsics... ";

  std::string response_message;

  // check if the distortion model is found in kCameraDistortionModels
  bool distortion_model_err = std::find(std::begin(seerep_server_constants::kCameraDistortionModels),
                                        std::end(seerep_server_constants::kCameraDistortionModels),
                                        camintrinsics->distortion_model().c_str()) ==
                              std::end(seerep_server_constants::kCameraDistortionModels);

  bool distortion_mat_err = camintrinsics->distortion_size() < 1;

  bool intrinsics_mat_err = camintrinsics->intrinsic_matrix_size() != 9 || camintrinsics->intrinsic_matrix()[0] == 0 ||
                            camintrinsics->intrinsic_matrix()[4] == 0;

  bool rectification_mat_err = camintrinsics->rectification_matrix_size() != 9;

  bool projection_mat_err = camintrinsics->projection_matrix_size() != 12;

  if (distortion_mat_err || distortion_model_err || intrinsics_mat_err || rectification_mat_err || projection_mat_err)
  {
    response_message = "";
    if (distortion_mat_err)
    {
      response_message += "The distortion matrix is not set. ";
    }
    if (distortion_model_err)
    {
      response_message += "The distortion model is not set or invalid. ";
    }
    if (intrinsics_mat_err)
    {
      response_message += "Instrinsics matrix dimensions are incorrect or the focal length values are "
                          "invalid. ";
    }
    if (rectification_mat_err)
    {
      response_message += "Rectification matrix dimensions are incorrect. ";
    }
    if (projection_mat_err)
    {
      response_message += "Projection matrix dimensions are incorrect.";
    }
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << response_message;
    seerep_server_util::createResponsePb(response_message, seerep::pb::ServerResponse::FAILURE, response);
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, response_message);
  }

  if (!camintrinsics->header().uuid_project().empty())
  {
    std::string grpc_msg = "Added Camera Intrinsics.";

    if (camintrinsics->maximum_viewing_distance() == 0)
    {
      std::string msg = "Max Viewing Distance is the distance to the farthest object in view and is used to compute a "
                        "frustrum of the camera's view. This value is not set. Using default value of 0.";
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;

      grpc_msg += msg;
    }

    boost::uuids::uuid uuid;
    try
    {
      boost::uuids::string_generator gen;
      uuid = gen(camintrinsics->header().uuid_project());

      camIntrinsicsPb->addData(*camintrinsics);

      seerep_server_util::createResponsePb(grpc_msg, seerep::pb::ServerResponse::SUCCESS, response);

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

grpc::Status PbCameraIntrinsicsService::GetCameraIntrinsics(grpc::ServerContext* context,
                                                            const seerep::pb::CameraIntrinsicsQuery* camintrinsicsQuery,
                                                            seerep::pb::CameraIntrinsics* response)
{
  (void)context;  // ignore that variable without causing warnings
  try
  {
    std::optional<seerep::pb::CameraIntrinsics> result;
    result = camIntrinsicsPb->getData(*camintrinsicsQuery);

    if (result)
    {
      *response = result.value();
    }
  }
  catch (std::runtime_error const& e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
    // also catching core doesn't have project with uuid error
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << e.what();
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
} /* namespace seerep_server */
