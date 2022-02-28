#include "seerep-core/image-overview.h"

namespace seerep_core
{
ImageOverview::ImageOverview(std::shared_ptr<seerep_pb_io::ImageIO> hdf5_io,
                             std::shared_ptr<seerep_core::TFOverview> tfOverview, std::string frameId)
  : m_hdf5_io(hdf5_io), m_tfOverview(tfOverview), m_frameId(frameId), m_data_count(0)
{
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

      uint64_t id = m_data_count++;

      auto img = std::make_shared<Image>(m_hdf5_io, id, uuid);

      addImageToIndices(img);
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

  if (m_data_count > 0)
  {
    // check if the data has now some tf for adding it to spatial rtree
    tryAddingDataWithMissingTF();

    // space
    std::vector<AabbHierarchy::AabbIdPair> resultRt = querySpatial(query);
    // time
    std::vector<AabbHierarchy::AabbTimeIdPair> resultTime = queryTemporal(query);
    // semantic
    std::set<uint64_t> resultSemantic = querySemantic(query);

    std::vector<uint64_t> resultIntersection = intersectQueryResults(resultRt, resultTime, resultSemantic);

    for (auto& r : resultIntersection)
    {
      std::optional<seerep::Image> img = m_datasets.at(r)->getData(query);

      if (img)
      {
        std::cout << "checked " << img.value().data() << std::endl;
        result.push_back(img);
      }
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

std::set<uint64_t> ImageOverview::querySemantic(const seerep::Query& query)
{
  std::set<uint64_t> result;
  // find the queried label in the label-imageID-map
  for (std::string labelquery : query.label())
  {
    // add all imageIDs to result set
    for (uint64_t ids : m_label.find(labelquery)->second)
    {
      result.insert(ids);
    }
  }
  return result;
}

std::vector<uint64_t> ImageOverview::intersectQueryResults(std::vector<AabbHierarchy::AabbIdPair> rt_result,
                                                           std::vector<AabbHierarchy::AabbTimeIdPair> timetree_result,
                                                           std::set<uint64_t> semanticResult)
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
  std::set<uint64_t> resultSpatioTemporal;
  std::set_intersection(idsSpatial.begin(), idsSpatial.end(), idsTemporal.begin(), idsTemporal.end(),
                        std::inserter(resultSpatioTemporal, resultSpatioTemporal.begin()));

  std::vector<uint64_t> result;
  std::set_intersection(resultSpatioTemporal.begin(), resultSpatioTemporal.end(), semanticResult.begin(),
                        semanticResult.end(), std::back_inserter(result));

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
  uint64_t id = m_data_count++;
  auto img = std::make_shared<Image>(m_hdf5_io, image, id, uuid);
  addImageToIndices(img);

  return uuid;
}

/*
 * Data without tf cannot be added to rtree
 * check if tf is now available and add data to spatial rtree
 */
void ImageOverview::tryAddingDataWithMissingTF()
{
  for (std::vector<std::shared_ptr<seerep_core::Image>>::iterator it = m_dataWithMissingTF.begin();
       it != m_dataWithMissingTF.end();
       /*it++*/ /*<-- increment in loop itself!*/)
  {
    int64_t timeSecs, timeNanos;
    (*it)->getTime(timeSecs, timeNanos);
    if (m_tfOverview->canTransform((*it)->getFrameId(), m_frameId, timeSecs, timeNanos))
    {
      m_rt.insert(std::make_pair(m_tfOverview->transformAABB((*it)->getAABB(), (*it)->getFrameId(), m_frameId, timeSecs,
                                                             timeNanos),
                                 (*it)->getID()));
      it = m_dataWithMissingTF.erase(it);
    }
    else
    {
      it++;
    }
  }
}

void ImageOverview::addImageToIndices(std::shared_ptr<seerep_core::Image> img)
{
  m_datasets.insert(std::make_pair(img->getID(), img));
  int64_t timeSecs, timeNanos;
  img->getTime(timeSecs, timeNanos);
  if (m_tfOverview->canTransform(img->getFrameId(), m_frameId, timeSecs, timeNanos))
  {
    m_rt.insert(std::make_pair(
        m_tfOverview->transformAABB(img->getAABB(), img->getFrameId(), m_frameId, timeSecs, timeNanos), img->getID()));
  }
  else
  {
    m_dataWithMissingTF.push_back(img);
  }
  m_timetree.insert(std::make_pair(img->getAABBTime(), img->getID()));

  std::unordered_set<std::string> labels = img->getLabels();

  for (std::string label : labels)
  {
    // check if label already exists
    std::unordered_map<std::string, std::vector<uint64_t>>::iterator labelmapentry = m_label.find(label);
    if (labelmapentry != m_label.end())
    {
      // label already exists, add id of image to the vector
      labelmapentry->second.push_back(img->getID());
    }
    else
    {
      // label doesn't already exist. Create new pair of label and vector of image ids
      m_label.insert(std::make_pair(label, std::vector<uint64_t>{ img->getID() }));
    }
  }
}

} /* namespace seerep_core */
