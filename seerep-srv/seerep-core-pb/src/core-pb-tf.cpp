#include "seerep-core-pb/core-pb-tf.h"

namespace seerep_core_pb
{
// constructor when data received and stored to hdf5
CorePbTf::CorePbTf(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
  for (seerep_core_msgs::ProjectInfo projectInfo : m_seerepCore->getProjects())
  {
    auto hdf5file = m_seerepCore->getHdf5File(projectInfo.uuid);
    auto hdf5fileMutex = m_seerepCore->getHdf5FileMutex(projectInfo.uuid);
    auto tfIo = std::make_shared<seerep_hdf5_pb::Hdf5PbTf>(hdf5file, hdf5fileMutex);

    m_hdf5IoMap.insert(std::make_pair(projectInfo.uuid, tfIo));
  }
}

CorePbTf::~CorePbTf()
{
}

std::optional<seerep::TransformStamped> CorePbTf::getData(const seerep::TransformStampedQuery& query)
{
  std::cout << "loading tf from tfs/" << std::endl;
  boost::uuids::string_generator gen;
  seerep_core_msgs::QueryTf queryTf;
  queryTf.childFrameId = query.child_frame_id();
  queryTf.parentFrameId = query.header().frame_id();
  queryTf.project = gen(query.header().uuid_project());
  queryTf.timestamp.seconds = query.header().stamp().seconds();
  queryTf.timestamp.nanos = query.header().stamp().nanos();
  std::optional<geometry_msgs::TransformStamped> result = m_seerepCore->getTF(queryTf);

  if (result)
  {
    return seerep_ros_conversions::toProto(result.value());
  }
  else
  {
    return std::nullopt;
  }
}

void CorePbTf::addData(const seerep::TransformStamped& tf)
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid projectuuid = gen(tf.header().uuid_project());

  // write to hdf5
  m_hdf5IoMap.at(projectuuid)->writeTransformStamped(tf);

  // add to seerep-core
  m_seerepCore->addTF(seerep_ros_conversions::toROS(tf), projectuuid);
}

std::vector<std::string> CorePbTf::getFrames(const boost::uuids::uuid& projectuuid)
{
  return m_seerepCore->getFrames(projectuuid);
}

}  // namespace seerep_core_pb
