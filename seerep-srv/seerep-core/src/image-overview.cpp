#include "seerep-core/image-overview.h"

namespace seerep_core
{
ImageOverview::ImageOverview()
{
}
ImageOverview::ImageOverview(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io) : m_hdf5_io(hdf5_io), data_count(0)
{
  coordinatesystem = "test";

  recreateDatasets();
}
ImageOverview::~ImageOverview()
{
}

void ImageOverview::recreateDatasets()
{
  std::vector<std::string> imgs = m_hdf5_io->getGroupDatasets("images");
  for (auto name : imgs)
  {
    std::cout << "found " << name << " in HDF5 file." << std::endl;

    try
    {
      boost::uuids::string_generator gen;
      boost::uuids::uuid uuid = gen(name);

      uint64_t id = data_count++;

      auto img = std::make_shared<Image>(coordinatesystem, m_hdf5_io, id, uuid);
      m_datasets.insert(std::make_pair(id, img));
      m_rt.insert(std::make_pair(img->getAABB(), img->getID()));
    }
    catch (const std::runtime_error& e)
    {
      std::cout << e.what() << std::endl;
    }
  }
}

std::vector<std::optional<seerep::Image>> ImageOverview::getData(const seerep::Boundingbox& bb)
{
  std::vector<std::optional<seerep::Image>> result;

  AabbHierarchy::AABB aabb(AabbHierarchy::Point(bb.point_min().x(), bb.point_min().y(), bb.point_min().z()),
                           AabbHierarchy::Point(bb.point_max().x(), bb.point_max().y(), bb.point_max().z()));

  std::vector<AabbHierarchy::AabbIdPair> rt_result;

  // do the semantic query first and extend this query parameter by check if id is in semantic result
  m_rt.query(boost::geometry::index::intersects(aabb), std::back_inserter(rt_result));

  for (auto& r : rt_result)
  {
    std::optional<seerep::Image> img = m_datasets.at(r.second)->getData(bb);

    if (img)
    {
      std::cout << "checked " << img.value().data() << std::endl;
      result.push_back(img);
    }
  }

  return result;
}

void ImageOverview::addDataset(const seerep::Image& image)
{
  boost::uuids::uuid uuid = boost::uuids::random_generator()();
  uint64_t id = data_count++;
  auto img = std::make_shared<Image>(coordinatesystem, m_hdf5_io, image, id, uuid);
  m_datasets.insert(std::make_pair(id, img));
  m_rt.insert(std::make_pair(img->getAABB(), img->getID()));
}

// void ImageOverview::addDatasetLabeled(const seerep::ImageLabeled& imagelabeled)
// {
//   uint64_t id = data_count++;
//   auto pc = std::make_shared<Image>(coordinatesystem, m_hdf5_io, imagelabeled, id);
//   m_datasets.insert(std::make_pair(id, pc));

//   m_rt.insert(std::make_pair(pc->getAABB(), pc->getID()));
// }

} /* namespace seerep_core */
