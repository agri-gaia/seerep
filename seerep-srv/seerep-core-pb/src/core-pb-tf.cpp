#include "seerep-core-pb/core-pb-tf.h"

namespace seerep_core_pb
{
// constructor when data received and stored to hdf5
CorePbTf::CorePbTf(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
  for (seerep_core_msgs::ProjectInfo projectInfo : m_seerepCore->getProjects())
  {
    getFileAccessorFromCore(projectInfo.uuid);
  }
}

CorePbTf::~CorePbTf()
{
}

std::optional<seerep::pb::TransformStamped> CorePbTf::getData(const seerep::pb::TransformStampedQuery& query)
{
  std::cout << "loading tf from tfs/" << std::endl;
  seerep_core_msgs::QueryTf queryTf = CorePbConversion::fromPb(query);

  std::optional<geometry_msgs::TransformStamped> result = m_seerepCore->getTF(queryTf);

  if (result)
  {
    return seerep_ros_conversions_pb::toProto(result.value());
  }
  else
  {
    return std::nullopt;
  }
}

void CorePbTf::addData(const seerep::pb::TransformStamped& tf)
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid projectuuid = gen(tf.header().uuid_project());

  // write to hdf5
  auto hdf5io = getHdf5(projectuuid);
  hdf5io->writeTransformStamped(tf);

  // add to seerep-core
  m_seerepCore->addTF(seerep_ros_conversions_pb::toROS(tf), projectuuid);
}

std::vector<std::string> CorePbTf::getFrames(const boost::uuids::uuid& projectuuid)
{
  return m_seerepCore->getFrames(projectuuid);
}

void CorePbTf::getFileAccessorFromCore(boost::uuids::uuid project)
{
  auto hdf5file = m_seerepCore->getHdf5File(project);
  auto hdf5fileMutex = m_seerepCore->getHdf5FileMutex(project);
  auto tfIo = std::make_shared<seerep_hdf5_pb::Hdf5PbTf>(hdf5file, hdf5fileMutex);
  m_hdf5IoMap.insert(std::make_pair(project, tfIo));
}

std::shared_ptr<seerep_hdf5_pb::Hdf5PbTf> CorePbTf::getHdf5(boost::uuids::uuid project)
{
  // find the project based on its uuid
  auto hdf5io = m_hdf5IoMap.find(project);
  // if project was found return hdf5 accessor
  if (hdf5io != m_hdf5IoMap.end())
  {
    return hdf5io->second;
  }
  // if not found ask core
  else
  {
    // this throws an exeption if core has no project with the uuid
    getFileAccessorFromCore(project);
    // if getFileAccessorFromCore didn't throw an error, find project and return pointer
    return m_hdf5IoMap.find(project)->second;
  };
}

}  // namespace seerep_core_pb
