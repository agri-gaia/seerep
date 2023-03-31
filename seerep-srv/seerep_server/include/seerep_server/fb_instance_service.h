#ifndef SEEREP_SERVER_FB_INSTANCE_SERVICE_H_
#define SEEREP_SERVER_FB_INSTANCE_SERVICE_H_

// seerep
#include <seerep_com/instance_service.grpc.fb.h>
#include <seerep_core/core.h>
#include <seerep_core_fb/core_fb_instance.h>

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_server
{
class FbInstanceService final : public seerep::fb::InstanceService::Service
{
public:
  FbInstanceService(std::shared_ptr<seerep_core::Core> seerepCore);

  grpc::Status GetInstances(grpc::ServerContext* context,
                            const flatbuffers::grpc::Message<seerep::fb::QueryInstance>* request,
                            flatbuffers::grpc::Message<seerep::fb::UuidsPerProject>* response);

private:
  std::shared_ptr<seerep_core_fb::CoreFbInstance> m_instanceFb;
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_FB_INSTANCE_SERVICE_H_
