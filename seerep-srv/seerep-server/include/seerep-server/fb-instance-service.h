#ifndef SEEREP_SERVER_FB_INSTANCE_SERVICE_H_
#define SEEREP_SERVER_FB_INSTANCE_SERVICE_H_

// seerep
#include <seerep-com/instance_service.grpc.fb.h>
#include <seerep-core-fb/core-fb-instance.h>
#include <seerep-core/core.h>

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
};

} /* namespace seerep_server */
#endif  // SEEREP_SERVER_FB_INSTANCE_SERVICE_H_
