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

boost::uuids::uuid
CoreFbTf::deleteHdf5(const seerep::fb::TransformStampedIntervalQuery& tfInterval)
{
  auto projectuuid = boost::lexical_cast<boost::uuids::uuid>(
      tfInterval.transform_stamped_query()->header()->uuid_project()->str());

  // delete from hdf5 files
  auto hdf5io = CoreFbGeneral::getHdf5(projectuuid, m_seerepCore, m_hdf5IoMap);

  // non static frames
  hdf5io->deleteTransformStamped(
      tfInterval.transform_stamped_query()->header()->frame_id()->str(),
      tfInterval.transform_stamped_query()->child_frame_id()->str(), false,
      *tfInterval.time_interval()->time_min(),
      *tfInterval.time_interval()->time_max());

  // static frames
  hdf5io->deleteTransformStamped(
      tfInterval.transform_stamped_query()->header()->frame_id()->str(),
      tfInterval.transform_stamped_query()->child_frame_id()->str(), true,
      *tfInterval.time_interval()->time_min(),
      *tfInterval.time_interval()->time_max());

  return projectuuid;
}
void CoreFbTf::reinitializeTFs(const boost::uuids::uuid& projectuuid)
{
  // recreate the tf buffer
  m_seerepCore->reinitializeTFs(projectuuid);
}

}  // namespace seerep_core_fb
