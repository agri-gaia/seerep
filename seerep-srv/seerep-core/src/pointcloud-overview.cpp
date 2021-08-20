#include "seerep-core/pointcloud-overview.h"

namespace seerep_core
{
PointcloudOverview::PointcloudOverview(std::string projectname) : projectname(projectname)
{
  coordinatesystem = "test";
  HighFive::File hdf5_file(projectname + ".h5", HighFive::File::ReadWrite | HighFive::File::Create);
  // HighFive::File::ReadOnly);
  // HighFive::File::ReadWrite | HighFive::File::Create | HighFive::File::Truncate);
  hdf5_io = std::make_shared<seerep_hdf5::SeerepHDF5IO>(hdf5_file);
  recreateDatasets();
}
PointcloudOverview::~PointcloudOverview()
{
}

void PointcloudOverview::recreateDatasets()
{
  std::vector<std::string> pcs = hdf5_io->getGroupDatasets("pointclouds");
  for (auto name : pcs)
  {
    std::cout << "found " << name << " in HDF5 file." << std::endl;

    auto pc = std::make_shared<Pointcloud>(coordinatesystem, hdf5_io, name);
    datasets.insert(std::make_pair(name, pc));
  }
}

std::vector<std::unique_ptr<seerep::PointCloud2>> PointcloudOverview::getData(const std::string& id,
                                                                              const seerep::Boundingbox& bb)
{
  std::vector<std::unique_ptr<seerep::PointCloud2>> result;
  for (auto& dataset : datasets)
  {
    std::optional<seerep::PointCloud2> pc = dataset.second->getData(id, bb);

    if (pc)
    {
      result.push_back(std::make_unique<seerep::PointCloud2>(pc.value()));
    }
  }

  return result;
}

void PointcloudOverview::addDataset(const std::string& id, const seerep::PointCloud2& pointcloud2)
{
  auto pc = std::make_shared<Pointcloud>(coordinatesystem, hdf5_io, pointcloud2, id);
  datasets.insert(std::make_pair(id, pc));
}

} /* namespace seerep_core */
