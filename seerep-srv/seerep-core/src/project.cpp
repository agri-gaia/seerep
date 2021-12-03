#include "seerep-core/project.h"

namespace seerep_core
{
Project::Project(boost::uuids::uuid& uuid, std::string path) : m_id(uuid), m_path(path)
{
  m_coordinatesystem = "test";

  createHdf5Io(m_id, m_path);
  m_pointcloudOverview = seerep_core::PointcloudOverview(m_hdf5_io);

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
  return m_pointcloudOverview.getData(query);
}

std::vector<std::optional<seerep::Image>> Project::getImage(const seerep::Query& query)
{
  return m_imageOverview.getData(query);
}

void Project::createHdf5Io(boost::uuids::uuid& uuid, std::string path)
{
  HighFive::File hdf5_file(path, HighFive::File::ReadWrite | HighFive::File::Create);
  m_hdf5_io = std::make_shared<seerep_hdf5::SeerepHDF5IO>(hdf5_file);
}

void Project::recreateDatatypes()
{
  std::vector<std::string> datatypeNames = m_hdf5_io->getGroupDatasets("");
  for (auto datatypeName : datatypeNames)
  {
    std::cout << "found datatype" << datatypeName << " in HDF5 file." << std::endl;

    if (datatypeName == "pointclouds")
    {
      m_pointcloudOverview = seerep_core::PointcloudOverview(m_hdf5_io);
    }
    else if (datatypeName == "images")
    {
      m_imageOverview = seerep_core::ImageOverview(m_hdf5_io);
    }
    else
    {
      std::cout << "unknown datatype" << std::endl;
    }
  }
}

void Project::addPointCloud(const seerep::PointCloud2& pointcloud2)
{
  m_pointcloudOverview.addDataset(pointcloud2);
}

void Project::addPointCloudLabeled(const seerep::PointCloud2Labeled& pointcloud2Labeled)
{
  m_pointcloudOverview.addDatasetLabeled(pointcloud2Labeled);
}

} /* namespace seerep_core */
