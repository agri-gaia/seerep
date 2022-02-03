#include "seerep-core/pointcloud-overview.h"

namespace seerep_core
{
PointcloudOverview::PointcloudOverview()
{
}
PointcloudOverview::PointcloudOverview(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io)
  : m_hdf5_io(hdf5_io), data_count(0)
{
  coordinatesystem = "test";

  recreateDatasets();
}
PointcloudOverview::~PointcloudOverview()
{
}

void PointcloudOverview::recreateDatasets()
{
  // std::vector<std::string> pcs = m_hdf5_io->getGroupDatasets("pointclouds");
  std::map<std::string, HighFive::Group> clouds = m_hdf5_io->getPointClouds();

  // TODO clear trees
  data_count = 0;

  for (auto cloud : clouds)
  {
    std::cout << "found " << cloud.first << " in HDF5 file." << std::endl;

    // m_datasets.insert(std::make_pair(id, pc));
    // m_rt.insert(std::make_pair(pc->getAABB(), pc->getID()));
  }
}

std::vector<std::optional<seerep::PointCloud2>> PointcloudOverview::getData(const seerep::Query& query)
{
  std::vector<std::optional<seerep::PointCloud2>> result;

  AabbHierarchy::AABB aabb(
      AabbHierarchy::Point(query.boundingbox().point_min().x(), query.boundingbox().point_min().y(),
                           query.boundingbox().point_min().z()),
      AabbHierarchy::Point(query.boundingbox().point_max().x(), query.boundingbox().point_max().y(),
                           query.boundingbox().point_max().z()));

  std::vector<AabbHierarchy::AabbIdPair> rt_result;

  // do the semantic query first and extend this query parameter by check if id is in semantic result
  m_rt.query(boost::geometry::index::intersects(aabb), std::back_inserter(rt_result));

  for (auto& r : rt_result)
  {
    std::optional<seerep::PointCloud2> pc = m_datasets.at(r.second)->getData(query);

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
  auto pc = std::make_shared<Pointcloud>(coordinatesystem, m_hdf5_io, pointcloud2, id);
  m_datasets.insert(std::make_pair(id, pc));
  m_rt.insert(std::make_pair(pc->getAABB(), pc->getID()));
}

// void PointcloudOverview::addDatasetLabeled(const seerep::PointCloud2Labeled& pointcloud2labeled)
// {
//   uint64_t id = data_count++;
//   auto pc = std::make_shared<Pointcloud>(coordinatesystem, m_hdf5_io, pointcloud2labeled, id);
//   m_datasets.insert(std::make_pair(id, pc));

//   m_rt.insert(std::make_pair(pc->getAABB(), pc->getID()));
// }

} /* namespace seerep_core */
