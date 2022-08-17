#include "seerep-core-fb/core-fb-instance.h"

namespace seerep_core_fb
{
CoreFbInstance::CoreFbInstance(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
}

CoreFbInstance::~CoreFbInstance()
{
}

void CoreFbInstance::getInstances(const flatbuffers::grpc::Message<seerep::fb::QueryInstance>* request,
                                  flatbuffers::grpc::Message<seerep::fb::UuidsPerProject>* response)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "getting instances";

  seerep_core_msgs::Query queryCore = seerep_core_fb::CoreFbConversion::fromFb(request->GetRoot());

  seerep_core_msgs::QueryResult result = m_seerepCore->getInstances(queryCore);

  *response = seerep_core_fb::CoreFbConversion::toFb(result);
}

}  // namespace seerep_core_fb
