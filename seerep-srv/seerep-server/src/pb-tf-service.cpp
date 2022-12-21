#include "seerep-server/pb-tf-service.h"

namespace seerep_server
{
PbTfService::PbTfService(std::shared_ptr<seerep_core::Core> seerepCore)
  : tfPb(std::make_shared<seerep_core_pb::CorePbTf>(seerepCore))
{
}

grpc::Status PbTfService::TransferTransformStamped(grpc::ServerContext* context,
                                                   const seerep::TransformStamped* transform,
                                                   seerep::ServerResponse* response)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "received transform... ";

  if (!transform->header().uuid_project().empty())
  {
    boost::uuids::uuid uuid;
    try
    {
      boost::uuids::string_generator gen;
      uuid = gen(transform->header().uuid_project());

      tfPb->addData(*transform);

      seerep_server_util::createResponsePb("added transform", seerep::ServerResponse::TRANSMISSION_STATE_SUCCESS,
                                           response);

      return grpc::Status::OK;
    }
    catch (std::runtime_error const& e)
    {
      // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
      // also catching core doesn't have project with uuid error
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
      seerep_server_util::createResponsePb(std::string(e.what()), seerep::ServerResponse::TRANSMISSION_STATE_FAILURE,
                                           response);

      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
    }
    catch (const std::exception& e)
    {
      // specific handling for all exceptions extending std::exception, except
      // std::runtime_error which is handled explicitly
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
      seerep_server_util::createResponsePb(std::string(e.what()), seerep::ServerResponse::TRANSMISSION_STATE_FAILURE,
                                           response);
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
    }
    catch (...)
    {
      // catch any other errors (that we have no information about)
      std::string msg = "Unknown failure occurred. Possible memory corruption";
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
      seerep_server_util::createResponsePb(msg, seerep::ServerResponse::TRANSMISSION_STATE_FAILURE, response);
      return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
    }
  }
  else
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning) << "project_uuid is empty!";
    seerep_server_util::createResponsePb("project_uuid is empty!", seerep::ServerResponse::TRANSMISSION_STATE_FAILURE,
                                         response);

    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "project_uuid is empty!");
  }
}

grpc::Status PbTfService::GetFrames(grpc::ServerContext* context, const seerep::FrameQuery* frameQuery,
                                    seerep::FrameInfos* response)
{
  (void)context;  // ignore that variable without causing warnings
  boost::uuids::uuid uuid;
  try
  {
    boost::uuids::string_generator gen;
    uuid = gen(frameQuery->project_uuid());

    for (auto framename : tfPb->getFrames(uuid))
    {
      response->add_frames(framename);
    }
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

grpc::Status PbTfService::GetTransformStamped(grpc::ServerContext* context,
                                              const seerep::TransformStampedQuery* transformQuery,
                                              seerep::TransformStamped* response)
{
  (void)context;  // ignore that variable without causing warnings
  try
  {
    auto result = tfPb->getData(*transformQuery);
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
