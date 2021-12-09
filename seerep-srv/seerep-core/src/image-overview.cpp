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
      m_timetree.insert(std::make_pair(img->getAABBTime(), img->getID()));
    }
    catch (const std::runtime_error& e)
    {
      std::cout << e.what() << std::endl;
    }
  }
}

std::vector<std::optional<seerep::Image>> ImageOverview::getData(const seerep::Query& query)
{
  std::vector<std::optional<seerep::Image>> result;

  // space
  std::vector<AabbHierarchy::AabbIdPair> resultRt = querySpatial(query);
  // time
  std::vector<AabbHierarchy::AabbTimeIdPair> resultTime = queryTemporal(query);

  std::vector<uint64_t> resultIntersection = intersectQueryResults(resultRt, resultTime);

  for (auto& r : resultIntersection)
  {
    std::optional<seerep::Image> img = m_datasets.at(r)->getData(query);

    if (img)
    {
      std::cout << "checked " << img.value().data() << std::endl;
      result.push_back(img);
    }
  }

  return result;
}

std::vector<AabbHierarchy::AabbIdPair> ImageOverview::querySpatial(const seerep::Query& query)
{
  AabbHierarchy::AABB aabb(
      AabbHierarchy::Point(query.boundingbox().point_min().x(), query.boundingbox().point_min().y(),
                           query.boundingbox().point_min().z()),
      AabbHierarchy::Point(query.boundingbox().point_max().x(), query.boundingbox().point_max().y(),
                           query.boundingbox().point_max().z()));
  std::vector<AabbHierarchy::AabbIdPair> rt_result;
  m_rt.query(boost::geometry::index::intersects(aabb), std::back_inserter(rt_result));
  return rt_result;
}

std::vector<uint64_t> ImageOverview::intersectQueryResults(std::vector<AabbHierarchy::AabbIdPair> rt_result,
                                                           std::vector<AabbHierarchy::AabbTimeIdPair> timetree_result)
{
  std::set<uint64_t> idsSpatial;
  for (auto it = std::make_move_iterator(rt_result.begin()), end = std::make_move_iterator(rt_result.end()); it != end;
       ++it)
  {
    idsSpatial.insert(std::move(it->second));
  }
  std::set<uint64_t> idsTemporal;
  for (auto it = std::make_move_iterator(timetree_result.begin()), end = std::make_move_iterator(timetree_result.end());
       it != end; ++it)
  {
    idsTemporal.insert(std::move(it->second));
  }
  std::vector<uint64_t> result;
  std::set_intersection(idsSpatial.begin(), idsSpatial.end(), idsTemporal.begin(), idsTemporal.end(),
                        std::back_inserter(result));
  return result;
}

std::vector<AabbHierarchy::AabbTimeIdPair> ImageOverview::queryTemporal(const seerep::Query& query)
{
  AabbHierarchy::AabbTime aabbtime(AabbHierarchy::TimePoint(query.timeinterval().time_min()),
                                   AabbHierarchy::TimePoint(query.timeinterval().time_max()));

  std::vector<AabbHierarchy::AabbTimeIdPair> timetree_result;
  m_timetree.query(boost::geometry::index::intersects(aabbtime), std::back_inserter(timetree_result));
  return timetree_result;
}

boost::uuids::uuid ImageOverview::addDataset(const seerep::Image& image)
{
  boost::uuids::uuid uuid;
  if (image.header().uuid_msgs().empty())
  {
    uuid = boost::uuids::random_generator()();
  }
  else
  {
    boost::uuids::string_generator gen;
    uuid = gen(image.header().uuid_msgs());
  }

  uint64_t id = data_count++;
  auto img = std::make_shared<Image>(coordinatesystem, m_hdf5_io, image, id, uuid);
  m_datasets.insert(std::make_pair(id, img));
  m_rt.insert(std::make_pair(img->getAABB(), img->getID()));

  return uuid;
}

// void ImageOverview::addDatasetLabeled(const seerep::ImageLabeled& imagelabeled)
// {
//   uint64_t id = data_count++;
//   auto pc = std::make_shared<Image>(coordinatesystem, m_hdf5_io, imagelabeled, id);
//   m_datasets.insert(std::make_pair(id, pc));

//   m_rt.insert(std::make_pair(pc->getAABB(), pc->getID()));
// }

} /* namespace seerep_core */
