#include "seerep-core/project.h"

namespace seerep_core
{
Project::Project(boost::uuids::uuid& uuid, std::string path) : m_id(uuid), m_path(path)
{
  m_coordinatesystem = "test";

  createHdf5Io(m_id, m_path);

  m_projectname = m_hdf5_io->readProjectname();

  recreateDatatypes();
}

Project::Project(boost::uuids::uuid& uuid, std::string path, std::string projectname)
  : m_id(uuid), m_path(path), m_projectname(projectname)
{
  createHdf5Io(uuid, m_path);
  m_hdf5_io->writeProjectname(m_projectname);

  recreateDatatypes();
}

Project::~Project()
{
}

std::vector<std::optional<seerep::PointCloud2>> Project::getPointCloud(const seerep::Query& query)
{
  return m_pointcloudOverview->getData(query);
}

std::vector<std::optional<seerep::Image>> Project::getImage(const seerep::Query& query)
{
  return m_imageOverview->getData(query);
}

void Project::createHdf5Io(boost::uuids::uuid& uuid, std::string path)
{
  HighFive::File hdf5_file(path, HighFive::File::ReadWrite | HighFive::File::Create);
  m_hdf5_io = std::make_shared<seerep_hdf5::SeerepHDF5IO>(hdf5_file);
}

void Project::recreateDatatypes()
{
  m_pointcloudOverview = std::make_unique<seerep_core::PointcloudOverview>(m_hdf5_io);
  m_imageOverview = std::make_unique<seerep_core::ImageOverview>(m_hdf5_io);
  m_tfOverview = std::make_unique<seerep_core::TFOverview>(m_hdf5_io);

  std::vector<std::string> datatypeNames = m_hdf5_io->getGroupDatasets("");
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

} /* namespace seerep_core */
