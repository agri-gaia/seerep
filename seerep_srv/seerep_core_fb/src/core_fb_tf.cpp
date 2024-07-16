#include "seerep_core_fb/core_fb_tf.h"

namespace seerep_core_fb
{
// constructor when data received and stored to hdf5
CoreFbTf::CoreFbTf(std::shared_ptr<seerep_core::Core> seerepCore)
  : m_seerepCore(seerepCore)
{
  CoreFbGeneral::getAllFileAccessorFromCore(m_seerepCore, m_hdf5IoMap);
}

CoreFbTf::~CoreFbTf()
{
}

void CoreFbTf::getData(
    const seerep::fb::TransformStampedQuery& query,
    flatbuffers::grpc::Message<seerep::fb::TransformStamped>* response)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
      << "loading tf from tfs/";
  seerep_core_msgs::QueryTf queryTf = CoreFbConversion::fromFb(query);

  std::optional<geometry_msgs::TransformStamped> result =
      m_seerepCore->getTF(queryTf);

  if (result)
  {
    *response = seerep_ros_conversions_fb::toFlat(
        result.value(), query.header()->uuid_project()->str(), false);
  }
  else
  {
    return;
  }
}

void CoreFbTf::addData(const seerep::fb::TransformStamped& tf)
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid projectuuid = gen(tf.header()->uuid_project()->str());

  // write to hdf5
  auto hdf5io = CoreFbGeneral::getHdf5(projectuuid, m_seerepCore, m_hdf5IoMap);
  hdf5io->writeTransformStamped(tf);

  // add to seerep-core
  m_seerepCore->addTF(seerep_ros_conversions_fb::toROS(tf), projectuuid);
}

std::vector<std::string>
CoreFbTf::getFrames(const boost::uuids::uuid& projectuuid)
{
  return m_seerepCore->getFrames(projectuuid);
}

}  // namespace seerep_core_fb
