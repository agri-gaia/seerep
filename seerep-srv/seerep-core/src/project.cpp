#include "seerep-core/project.h"

namespace seerep_core
{
Project::Project(boost::uuids::uuid& uuid, std::string path) : id(uuid)
{
  coordinatesystem = "test";

  createHdf5Io(uuid, path);
  pointcloudOverview = seerep_core::PointcloudOverview(hdf5_io);

  projectname = hdf5_io->readProjectname();

  recreateDatatypes();
}

Project::Project(boost::uuids::uuid& uuid, std::string path, std::string projectname)
  : id(uuid), projectname(projectname)
{
  createHdf5Io(uuid, path);
  hdf5_io->writeProjectname(projectname);
  pointcloudOverview = seerep_core::PointcloudOverview(hdf5_io);
  recreateDatatypes();
}

Project::~Project()
{
}

std::vector<std::optional<seerep::PointCloud2>> Project::getPointCloud(const seerep::Boundingbox& bb)
{
  return pointcloudOverview.getData(bb);
}

void Project::createHdf5Io(boost::uuids::uuid& uuid, std::string path)
{
  HighFive::File hdf5_file(path, HighFive::File::ReadWrite | HighFive::File::Create);
  hdf5_io = std::make_shared<seerep_hdf5::SeerepHDF5IO>(hdf5_file);
}

void Project::recreateDatatypes()
{
  std::vector<std::string> datatypeNames = hdf5_io->getGroupDatasets("");
  for (auto datatypeName : datatypeNames)
  {
    std::cout << "found datatype" << datatypeName << " in HDF5 file." << std::endl;

    if (datatypeName == "pointclouds")
    {
      pointcloudOverview = seerep_core::PointcloudOverview(hdf5_io);
    }
    else
    {
      std::cout << "unknown datatype" << std::endl;
    }
  }
}

void Project::addPointCloud(const seerep::PointCloud2& pointcloud2)
{
  pointcloudOverview.addDataset(pointcloud2);
}

void Project::addPointCloudLabeled(const seerep::PointCloud2Labeled& pointcloud2Labeled)
{
  pointcloudOverview.addDatasetLabeled(pointcloud2Labeled);
}

} /* namespace seerep_core */
