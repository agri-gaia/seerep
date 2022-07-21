#include "seerep-core-fb/core-fb-tf.h"

namespace seerep_core_fb
{
// constructor when data received and stored to hdf5
CoreFbTf::CoreFbTf(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
  for (seerep_core_msgs::ProjectInfo projectInfo : m_seerepCore->getProjects())
  {
    getFileAccessorFromCore(projectInfo.uuid);
  }
}

CoreFbTf::~CoreFbTf()
{
}

void CoreFbTf::getData(const seerep::fb::TransformStampedQuery& query,
                       flatbuffers::grpc::Message<seerep::fb::TransformStamped>* response)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "loading tf from tfs/";
  seerep_core_msgs::QueryTf queryTf = CoreFbConversion::fromFb(query);

  std::optional<geometry_msgs::TransformStamped> result = m_seerepCore->getTF(queryTf);

  if (result)
  {
    *response = seerep_ros_conversions_fb::toFlat(result.value(), query.header()->uuid_project()->str());
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
  auto hdf5io = getHdf5(projectuuid);
  hdf5io->writeTransformStamped(tf);

  // add to seerep-core
  m_seerepCore->addTF(seerep_ros_conversions_fb::toROS(tf), projectuuid);
}

std::vector<std::string> CoreFbTf::getFrames(const boost::uuids::uuid& projectuuid)
{
  return m_seerepCore->getFrames(projectuuid);
}

void CoreFbTf::getFileAccessorFromCore(boost::uuids::uuid project)
{
  auto hdf5file = m_seerepCore->getHdf5File(project);
  auto hdf5fileMutex = m_seerepCore->getHdf5FileMutex(project);
  auto tfIo = std::make_shared<seerep_hdf5_fb::Hdf5FbTf>(hdf5file, hdf5fileMutex);
  m_hdf5IoMap.insert(std::make_pair(project, tfIo));
}

std::shared_ptr<seerep_hdf5_fb::Hdf5FbTf> CoreFbTf::getHdf5(boost::uuids::uuid project)
{
  // find the project based on its uuid
  auto hdf5io = m_hdf5IoMap.find(project);
  // if project was found add tf
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

}  // namespace seerep_core_fb
