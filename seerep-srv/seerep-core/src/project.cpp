#include "seerep-core/project.h"

namespace seerep_core
{
Project::Project(boost::uuids::uuid& uuid, std::string path) : m_id(uuid), m_path(path)
{
  createHdf5Io(m_id, m_path);

  m_projectname = m_ioGeneral->readProjectname();
  m_frameId = m_ioGeneral->readProjectFrameId();
  recreateDatatypes();
}

Project::Project(boost::uuids::uuid& uuid, std::string path, std::string projectname, std::string mapFrameId)
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

std::vector<std::optional<seerep::PointCloud2>> Project::getPointCloud(const seerep::Query& query)
{
  return m_pointcloudOverview->getData(m_tfOverview->transformQuery(query, m_frameId));
}

std::vector<std::optional<seerep::Image>> Project::getImage(const seerep::Query& query)
{
  return m_imageOverview->getData(m_tfOverview->transformQuery(query, m_frameId));
}

void Project::createHdf5Io(boost::uuids::uuid& uuid, std::string path)
{
  m_write_mtx = std::make_shared<std::mutex>();
  std::shared_ptr<HighFive::File> hdf5_file =
      std::make_shared<HighFive::File>(path, HighFive::File::ReadWrite | HighFive::File::Create);

  m_ioGeneral = std::make_shared<seerep_pb_io::GeneralIO>(hdf5_file, m_write_mtx);
  m_ioTf = std::make_shared<seerep_pb_io::TfIO>(hdf5_file, m_write_mtx);
  m_ioPointCloud = std::make_shared<seerep_pb_io::PointCloudIO>(hdf5_file, m_write_mtx);
  m_ioImage = std::make_shared<seerep_pb_io::ImageIO>(hdf5_file, m_write_mtx);
}

void Project::recreateDatatypes()
{
  m_tfOverview = std::make_shared<seerep_core::TFOverview>(m_ioTf);
  m_imageOverview = std::make_unique<seerep_core::ImageOverview>(m_ioImage, m_tfOverview, m_frameId);
  m_pointcloudOverview = std::make_unique<seerep_core::PointcloudOverview>(m_ioPointCloud);

  std::vector<std::string> datatypeNames = m_ioGeneral->getGroupDatasets("");
  for (auto datatypeName : datatypeNames)
  {
    std::cout << "found datatype" << datatypeName << " in HDF5 file." << std::endl;
  }
}

void Project::addPointCloud(const seerep::PointCloud2& pointcloud2)
{
  m_pointcloudOverview->addDataset(pointcloud2);
}

// void Project::addPointCloudLabeled(const seerep::PointCloud2Labeled& pointcloud2Labeled)
// {
//   m_pointcloudOverview->addDatasetLabeled(pointcloud2Labeled);
// }

boost::uuids::uuid Project::addImage(const seerep::Image& image)
{
  return m_imageOverview->addDataset(image);
}

void Project::addTF(const seerep::TransformStamped& tf)
{
  m_tfOverview->addDataset(tf);
}

std::optional<seerep::TransformStamped> Project::getTF(const seerep::TransformStampedQuery& transformQuery)
{
  return m_tfOverview->getData(transformQuery.header().stamp().seconds(), transformQuery.header().stamp().nanos(),
                               transformQuery.header().frame_id(), transformQuery.child_frame_id());
}

std::vector<std::string> Project::getFrames()
{
  return m_tfOverview->getFrames();
}

} /* namespace seerep_core */
