#include "seerep-core/core-project.h"

namespace seerep_core
{
CoreProject::CoreProject(const boost::uuids::uuid& uuid, const std::string path) : m_id(uuid), m_path(path)
{
  createHdf5Io(m_id, m_path);

  m_projectname = m_ioGeneral->readProjectname();
  m_frameId = m_ioGeneral->readProjectFrameId();
  recreateDatatypes();
}
CoreProject::CoreProject(const boost::uuids::uuid& uuid, const std::string path, const std::string projectname,
                         const std::string mapFrameId)
  : m_id(uuid), m_path(path), m_projectname(projectname), m_frameId(mapFrameId)
{
  createHdf5Io(m_id, m_path);
  m_ioGeneral->writeProjectname(m_projectname);
  m_ioGeneral->writeProjectFrameId(m_frameId);
  recreateDatatypes();
}
CoreProject::~CoreProject()
{
}

std::string CoreProject::getName()
{
  return m_projectname;
}
std::string CoreProject::getFrameId()
{
  return m_frameId;
}

seerep_core_msgs::QueryResultProject CoreProject::getPointCloud(const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResultProject result;
  result.projectUuid = m_id;
  result.dataUuids = m_pointcloudOverview->getData(m_tfOverview->transformQuery(query, m_frameId));
  return result;
}
seerep_core_msgs::QueryResultProject CoreProject::getImage(const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResultProject result;
  result.projectUuid = m_id;
  result.dataUuids = m_imageOverview->getData(m_tfOverview->transformQuery(query, m_frameId));
  return result;
}

void CoreProject::addPointCloud(const seerep_core_msgs::DatasetIndexable& pointcloud)
{
  m_pointcloudOverview->addDataset(pointcloud);
}
void CoreProject::addImage(const seerep_core_msgs::DatasetIndexable& image)
{
  m_imageOverview->addDataset(image);
}

void CoreProject::addImageLabels(std::vector<std::string>& labels, const boost::uuids::uuid& msgUuid)
{
  m_imageOverview->addImageLabels(labels, msgUuid);
}

void CoreProject::addTF(const geometry_msgs::TransformStamped& tf)
{
  m_tfOverview->addDataset(tf);
}
std::optional<geometry_msgs::TransformStamped> CoreProject::getTF(const seerep_core_msgs::QueryTf& transformQuery)
{
  return m_tfOverview->getData(transformQuery.timestamp.seconds, transformQuery.timestamp.nanos,
                               transformQuery.parentFrameId, transformQuery.childFrameId);
}
std::vector<std::string> CoreProject::getFrames()
{
  return m_tfOverview->getFrames();
}

std::shared_ptr<std::mutex> CoreProject::getHdf5FileMutex()
{
  return m_write_mtx;
}
std::shared_ptr<HighFive::File> CoreProject::getHdf5File()
{
  return m_hdf5_file;
}

void CoreProject::createHdf5Io(boost::uuids::uuid& uuid, std::string path)
{
  m_write_mtx = std::make_shared<std::mutex>();
  m_hdf5_file = std::make_shared<HighFive::File>(path, HighFive::File::ReadWrite | HighFive::File::Create);

  m_ioGeneral = std::make_shared<seerep_hdf5_core::Hdf5CoreGeneral>(m_hdf5_file, m_write_mtx);
  m_ioTf = std::make_shared<seerep_hdf5_core::Hdf5CoreTf>(m_hdf5_file, m_write_mtx);
  m_ioPointCloud = std::make_shared<seerep_hdf5_core::Hdf5CorePointCloud>(m_hdf5_file, m_write_mtx);
  m_ioImage = std::make_shared<seerep_hdf5_core::Hdf5CoreImage>(m_hdf5_file, m_write_mtx);
}
void CoreProject::recreateDatatypes()
{
  m_tfOverview = std::make_shared<seerep_core::CoreTf>(m_ioTf);
  m_imageOverview = std::make_unique<seerep_core::CoreImage>(m_ioImage, m_tfOverview, m_frameId);
  m_pointcloudOverview = std::make_unique<seerep_core::CorePointCloud>(m_ioPointCloud, m_tfOverview, m_frameId);

  std::vector<std::string> datatypeNames = m_ioGeneral->getGroupDatasets("");
  for (auto datatypeName : datatypeNames)
  {
    std::cout << "found datatype" << datatypeName << " in HDF5 file." << std::endl;
  }
}

} /* namespace seerep_core */
