#include "seerep-core/project.h"

namespace seerep_core
{
Project::Project(const boost::uuids::uuid& uuid, const std::string path) : m_id(uuid), m_path(path)
{
  createHdf5Io(m_id, m_path);

  m_projectname = m_ioGeneral->readProjectname();
  m_frameId = m_ioGeneral->readProjectFrameId();
  recreateDatatypes();
}

Project::Project(const boost::uuids::uuid& uuid, const std::string path, const std::string projectname,
                 const std::string mapFrameId)
  : m_id(uuid), m_path(path), m_projectname(projectname), m_frameId(mapFrameId)
{
  createHdf5Io(m_id, m_path);
  m_ioGeneral->writeProjectname(m_projectname);
  m_ioGeneral->writeProjectFrameId(m_frameId);

  recreateDatatypes();
}

Project::~Project()
{
}

std::string Project::getName()
{
  return m_projectname;
}

std::string Project::getFrameId()
{
  return m_frameId;
}

// seerep_core_msgs::QueryResultProject Project::getPointCloud(const seerep_core_msgs::Query& query)
// {
//   return m_pointcloudOverview->getData(m_tfOverview->transformQuery(query, m_frameId));
// }

seerep_core_msgs::QueryResultProject Project::getImage(const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResultProject result;
  result.projectUuid = m_id;
  result.dataUuids = m_imageOverview->getData(m_tfOverview->transformQuery(query, m_frameId));
  return result;
}

void Project::createHdf5Io(boost::uuids::uuid& uuid, std::string path)
{
  m_write_mtx = std::make_shared<std::mutex>();
  m_hdf5_file = std::make_shared<HighFive::File>(path, HighFive::File::ReadWrite | HighFive::File::Create);

  m_ioGeneral = std::make_shared<seerep_core_io::GeneralIOCore>(m_hdf5_file, m_write_mtx);
  m_ioTf = std::make_shared<seerep_core_io::TfIOCore>(m_hdf5_file, m_write_mtx);
  // m_ioPointCloud = std::make_shared<seerep_core_io::PointCloudIOCore>(hdf5_file, m_write_mtx);
  m_ioImage = std::make_shared<seerep_core_io::ImageIOCore>(m_hdf5_file, m_write_mtx);
}

void Project::recreateDatatypes()
{
  m_tfOverview = std::make_shared<seerep_core::TFOverview>(m_ioTf);
  m_imageOverview = std::make_unique<seerep_core::ImageOverview>(m_ioImage, m_tfOverview, m_frameId);
  // m_pointcloudOverview = std::make_unique<seerep_core::PointcloudOverview>(m_ioPointCloud);

  std::vector<std::string> datatypeNames = m_ioGeneral->getGroupDatasets("");
  for (auto datatypeName : datatypeNames)
  {
    std::cout << "found datatype" << datatypeName << " in HDF5 file." << std::endl;
  }
}

// void Project::addPointCloud(const seerep::PointCloud2& pointcloud2)
// {
//   m_pointcloudOverview->addDataset(pointcloud2);
// }

void Project::addImage(const seerep_core_msgs::DatasetIndexable& image)
{
  m_imageOverview->addDataset(image);
}

void Project::addTF(const geometry_msgs::TransformStamped& tf)
{
  m_tfOverview->addDataset(tf);
}

std::optional<geometry_msgs::TransformStamped> Project::getTF(const seerep_core_msgs::QueryTf& transformQuery)
{
  return m_tfOverview->getData(transformQuery.timestamp.seconds, transformQuery.timestamp.nanos,
                               transformQuery.parentFrameId, transformQuery.childFrameId);
}

std::vector<std::string> Project::getFrames()
{
  return m_tfOverview->getFrames();
}

std::shared_ptr<std::mutex> Project::getHdf5FileMutex()
{
  return m_write_mtx;
}
std::shared_ptr<HighFive::File> Project::getHdf5File()
{
  return m_hdf5_file;
}

} /* namespace seerep_core */
