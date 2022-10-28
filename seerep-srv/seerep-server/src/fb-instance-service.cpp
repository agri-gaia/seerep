#include "seerep-server/fb-instance-service.h"

namespace seerep_server
{
FbInstanceService::FbInstanceService(std::shared_ptr<seerep_core::Core> seerepCore)
  : m_instanceFb(std::make_shared<seerep_core_fb::CoreFbInstance>(seerepCore))
{
}

grpc::Status FbInstanceService::GetInstances(grpc::ServerContext* context,
                                             const flatbuffers::grpc::Message<seerep::fb::QueryInstance>* request,
                                             flatbuffers::grpc::Message<seerep::fb::UuidsPerProject>* response)
{
  (void)context;  // ignore that variable without causing warnings
  try
  {
    m_instanceFb->getInstances(request, response);
  }
  catch (std::runtime_error const& e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
    // also catching core doesn't have project with uuid error
    std::cout << e.what() << std::endl;
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
