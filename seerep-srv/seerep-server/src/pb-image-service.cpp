#include "seerep-server/pb-image-service.h"

namespace seerep_server
{
PbImageService::PbImageService(std::shared_ptr<seerep_core::Core> seerepCore)
  : imagePb(std::make_shared<seerep_core_pb::CorePbImage>(seerepCore))
{
}

grpc::Status PbImageService::GetImage(grpc::ServerContext* context, const seerep::pb::Query* request,
                                      grpc::ServerWriter<seerep::pb::Image>* writer)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
      << "sending images in bounding box min(" << request->boundingboxstamped().boundingbox().point_min().x() << "/"
      << request->boundingboxstamped().boundingbox().point_min().y() << "/"
      << request->boundingboxstamped().boundingbox().point_min().z() << "), max("
      << request->boundingboxstamped().boundingbox().point_max().x() << "/"
      << request->boundingboxstamped().boundingbox().point_max().y() << "/"
      << request->boundingboxstamped().boundingbox().point_max().z() << ")"
      << " and time interval (" << request->timeinterval().time_min().seconds() << "/"
      << request->timeinterval().time_max().seconds() << ")";

  std::vector<seerep::pb::Image> images;
  try
  {
    imagePb->getData(*request, writer);
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

grpc::Status PbImageService::TransferImage(grpc::ServerContext* context, const seerep::pb::Image* image,
                                           seerep::pb::ServerResponse* response)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "received image... ";

  if (!image->header().uuid_project().empty())
  {
    boost::uuids::uuid uuid;
    try
    {
      boost::uuids::string_generator gen;
      uuid = gen(image->header().uuid_project());

      boost::uuids::uuid uuidImg = imagePb->addData(*image);

      seerep_server_util::createResponsePb(boost::lexical_cast<std::string>(uuidImg),
                                           seerep::pb::ServerResponse::SUCCESS, response);

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

} /* namespace seerep_server */
