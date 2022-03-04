#include "seerep-core/image-overview.h"

namespace seerep_core
{
ImageOverview::ImageOverview(std::shared_ptr<seerep_core_io::ImageIOCore> hdf5_io,
                             std::shared_ptr<seerep_core::TFOverview> tfOverview, std::string frameId)
  : m_hdf5_io(hdf5_io), m_tfOverview(tfOverview), m_frameId(frameId)
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

      std::optional<seerep_core_msgs::DatasetIndexable> img =
          m_hdf5_io->readDataForIndices(seerep_core_io::ImageIOCore::HDF5_GROUP_IMAGE, name);

      if (img)
        addImageToIndices(img.value());
    }
    catch (const std::runtime_error& e)
    {
      std::cout << e.what() << std::endl;
    }
  }
}

std::vector<boost::uuids::uuid> ImageOverview::getData(const seerep_core_msgs::Query& query)
{
  std::vector<boost::uuids::uuid> result;

  // check if the data has now some tf for adding it to spatial rtree
  tryAddingDataWithMissingTF();

  // space
  std::vector<seerep_core_msgs::AabbIdPair> resultRt = querySpatial(query);
  // time
  std::vector<seerep_core_msgs::AabbTimeIdPair> resultTime = queryTemporal(query);
  // semantic
  std::set<boost::uuids::uuid> resultSemantic = querySemantic(query);

  return intersectQueryResults(resultRt, resultTime, resultSemantic);
}

std::vector<seerep_core_msgs::AabbIdPair> ImageOverview::querySpatial(const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::AABB aabb(seerep_core_msgs::Point(bg::get<bg::min_corner, 0>(query.boundingbox),
                                                      bg::get<bg::min_corner, 1>(query.boundingbox),
                                                      bg::get<bg::min_corner, 2>(query.boundingbox)),
                              seerep_core_msgs::Point(bg::get<bg::max_corner, 0>(query.boundingbox),
                                                      bg::get<bg::max_corner, 1>(query.boundingbox),
                                                      bg::get<bg::max_corner, 2>(query.boundingbox)));
  std::vector<seerep_core_msgs::AabbIdPair> rt_result;
  m_rt.query(boost::geometry::index::intersects(aabb), std::back_inserter(rt_result));
  return rt_result;
}

std::set<boost::uuids::uuid> ImageOverview::querySemantic(const seerep_core_msgs::Query& query)
{
  std::set<boost::uuids::uuid> result;
  // find the queried label in the label-imageID-map
  for (std::string labelquery : query.label)
  {
    // add all imageIDs to result set
    for (boost::uuids::uuid ids : m_label.find(labelquery)->second)
    {
      result.insert(ids);
    }
  }
  return result;
}

std::vector<boost::uuids::uuid>
ImageOverview::intersectQueryResults(std::vector<seerep_core_msgs::AabbIdPair> rt_result,
                                     std::vector<seerep_core_msgs::AabbTimeIdPair> timetree_result,
                                     std::set<boost::uuids::uuid> semanticResult)
{
  std::set<boost::uuids::uuid> idsSpatial;
  for (auto it = std::make_move_iterator(rt_result.begin()), end = std::make_move_iterator(rt_result.end()); it != end;
       ++it)
  {
    idsSpatial.insert(std::move(it->second));
  }
  std::set<boost::uuids::uuid> idsTemporal;
  for (auto it = std::make_move_iterator(timetree_result.begin()), end = std::make_move_iterator(timetree_result.end());
       it != end; ++it)
  {
    idsTemporal.insert(std::move(it->second));
  }
  std::set<boost::uuids::uuid> resultSpatioTemporal;
  std::set_intersection(idsSpatial.begin(), idsSpatial.end(), idsTemporal.begin(), idsTemporal.end(),
                        std::inserter(resultSpatioTemporal, resultSpatioTemporal.begin()));

  std::vector<boost::uuids::uuid> result;
  std::set_intersection(resultSpatioTemporal.begin(), resultSpatioTemporal.end(), semanticResult.begin(),
                        semanticResult.end(), std::back_inserter(result));

  return result;
}

std::vector<seerep_core_msgs::AabbTimeIdPair> ImageOverview::queryTemporal(const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::AabbTime aabbtime(seerep_core_msgs::TimePoint(query.timeinterval.timeMin.seconds),
                                      seerep_core_msgs::TimePoint(query.timeinterval.timeMax.seconds));

  std::vector<seerep_core_msgs::AabbTimeIdPair> timetree_result;
  m_timetree.query(boost::geometry::index::intersects(aabbtime), std::back_inserter(timetree_result));
  return timetree_result;
}

boost::uuids::uuid ImageOverview::addDataset(seerep_core_msgs::DatasetIndexable& image)
{
  // does the default constructor of uuid generate nil uuids?
  if (image.header.uuidData.is_nil())
  {
    image.header.uuidData = boost::uuids::random_generator()();
  }

  addImageToIndices(image);
  return image.header.uuidData;
}

/*
 * Data without tf cannot be added to rtree
 * check if tf is now available and add data to spatial rtree
 */
void ImageOverview::tryAddingDataWithMissingTF()
{
  for (std::vector<std::shared_ptr<seerep_core_msgs::DatasetIndexable>>::iterator it = m_dataWithMissingTF.begin();
       it != m_dataWithMissingTF.end();
       /*it++*/ /*<-- increment in loop itself!*/)
  {
    if (m_tfOverview->canTransform((*it)->header.frameId, m_frameId, (*it)->header.timestamp.seconds,
                                   (*it)->header.timestamp.nanos))
    {
      m_rt.insert(
          std::make_pair(m_tfOverview->transformAABB((*it)->boundingbox, (*it)->header.frameId, m_frameId,
                                                     (*it)->header.timestamp.seconds, (*it)->header.timestamp.nanos),
                         (*it)->header.uuidData));
      it = m_dataWithMissingTF.erase(it);
    }
    else
    {
      it++;
    }
  }
}

void ImageOverview::addImageToIndices(seerep_core_msgs::DatasetIndexable& img)
{
  if (m_tfOverview->canTransform(img.header.frameId, m_frameId, img.header.timestamp.seconds,
                                 img.header.timestamp.nanos))
  {
    m_rt.insert(std::make_pair(m_tfOverview->transformAABB(img.boundingbox, img.header.frameId, m_frameId,
                                                           img.header.timestamp.seconds, img.header.timestamp.nanos),
                               img.header.uuidData));
  }
  else
  {
    m_dataWithMissingTF.push_back(std::make_shared<seerep_core_msgs::DatasetIndexable>(img));
  }
  seerep_core_msgs::AabbTime aabbTime(seerep_core_msgs::TimePoint(img.header.timestamp.seconds),
                                      seerep_core_msgs::TimePoint(img.header.timestamp.seconds));
  m_timetree.insert(std::make_pair(aabbTime, img.header.uuidData));

  auto& labels = img.labels;

  for (std::string label : labels)
  {
    // check if label already exists
    std::unordered_map<std::string, std::vector<boost::uuids::uuid>>::iterator labelmapentry = m_label.find(label);
    if (labelmapentry != m_label.end())
    {
      // label already exists, add id of image to the vector
      labelmapentry->second.push_back(img.header.uuidData);
    }
    else
    {
      // label doesn't already exist. Create new pair of label and vector of image ids
      m_label.insert(std::make_pair(label, std::vector<boost::uuids::uuid>{ img.header.uuidData }));
    }
  }
}

} /* namespace seerep_core */
