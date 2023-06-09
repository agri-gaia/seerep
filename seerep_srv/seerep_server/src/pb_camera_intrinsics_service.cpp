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
