#include "seerep-core/pointcloud-overview.h"

namespace seerep_core
{
PointcloudOverview::PointcloudOverview()
{
}
PointcloudOverview::PointcloudOverview(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io)
  : hdf5_io(hdf5_io), data_count(0)
{
  coordinatesystem = "test";

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

    uint64_t id;
    std::istringstream(name) >> id;
    auto pc = std::make_shared<Pointcloud>(coordinatesystem, hdf5_io, id);
    datasets.insert(std::make_pair(id, pc));
  }
}

std::vector<std::optional<seerep::PointCloud2>> PointcloudOverview::getData(const seerep::Boundingbox& bb)
{
  std::vector<std::optional<seerep::PointCloud2>> result;

  AabbHierarchy::AABB aabb(AabbHierarchy::Point(bb.point_min().x(), bb.point_min().y(), bb.point_min().z()),
                           AabbHierarchy::Point(bb.point_max().x(), bb.point_max().y(), bb.point_max().z()));

  std::vector<AabbHierarchy::AabbIdPair> rt_result;

  // do the semantic query first and extend this query parameter by check if id is in semantic result
  rt.query(boost::geometry::index::intersects(aabb), std::back_inserter(rt_result));

  for (auto& r : rt_result)
  {
    std::optional<seerep::PointCloud2> pc = datasets.at(r.second)->getData(bb);

    if (pc)
    {
      std::cout << "checked " << pc.value().data() << std::endl;
      result.push_back(pc);
    }
  }

  return result;
}

void PointcloudOverview::addDataset(const seerep::PointCloud2& pointcloud2)
{
  uint64_t id = data_count++;
  auto pc = std::make_shared<Pointcloud>(coordinatesystem, hdf5_io, pointcloud2, id);
  datasets.insert(std::make_pair(id, pc));
  // rt.insert(std::make_pair(,pc))
}

void PointcloudOverview::addDatasetLabeled(const seerep::PointCloud2Labeled& pointcloud2labeled)
{
  uint64_t id = data_count++;
  auto pc = std::make_shared<Pointcloud>(coordinatesystem, hdf5_io, pointcloud2labeled, id);
  datasets.insert(std::make_pair(id, pc));
}

} /* namespace seerep_core */
