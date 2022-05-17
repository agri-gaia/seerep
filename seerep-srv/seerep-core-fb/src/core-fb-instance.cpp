#include "seerep-core-fb/core-fb-instance.h"

namespace seerep_core_fb
{
CoreFbInstance::CoreFbInstance(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
}

CoreFbInstance::~CoreFbInstance()
{
}

void CoreFbInstance::getInstances(const flatbuffers::grpc::Message<seerep::fb::Query>* request,
                                  flatbuffers::grpc::Message<seerep::fb::UuidsPerProject>* response)
{
  std::cout << "loading image from images/" << std::endl;
  seerep_core_msgs::Query queryCore = seerep_core_fb::CoreFbConversion::fromFb(*request->GetRoot());

  seerep_core_msgs::QueryResult result = m_seerepCore->getInstances(queryCore);

  *response = seerep_core_fb::CoreFbConversion::toFb(result);
}

}  // namespace seerep_core_fb
