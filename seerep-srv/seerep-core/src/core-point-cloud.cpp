#include "seerep-core/core-point-cloud.h"

namespace seerep_core
{
CorePointCloud::CorePointCloud(std::shared_ptr<seerep_hdf5_core::Hdf5CorePointCloud> hdf5_io,
                               std::shared_ptr<seerep_core::CoreTf> tfOverview, std::string frameId)
  : m_frameId(frameId), m_tfOverview(tfOverview), m_hdf5_io(hdf5_io)
{
  recreateDatasets();
}
CorePointCloud::~CorePointCloud()
{
}

void CorePointCloud::recreateDatasets()
{
  std::vector<std::string> datasets =
      m_hdf5_io->getGroupDatasets(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD);
  for (auto uuid : datasets)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "found " << uuid << " in HDF5 file.";

    try
    {
      std::optional<seerep_core_msgs::DatasetIndexable> dataset = m_hdf5_io->readPointCloud(uuid);

      if (dataset)
        addDatasetToIndices(dataset.value());
    }
    catch (const std::runtime_error& e)
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    }
  }
}

std::vector<boost::uuids::uuid> CorePointCloud::getData(const seerep_core_msgs::Query& query)
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

std::vector<seerep_core_msgs::AabbIdPair> CorePointCloud::querySpatial(const seerep_core_msgs::Query& query)
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

std::set<boost::uuids::uuid> CorePointCloud::querySemantic(const seerep_core_msgs::Query& query)
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
CorePointCloud::intersectQueryResults(std::vector<seerep_core_msgs::AabbIdPair> rt_result,
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

std::vector<seerep_core_msgs::AabbTimeIdPair> CorePointCloud::queryTemporal(const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::AabbTime aabbtime(seerep_core_msgs::TimePoint(query.timeinterval.timeMin.seconds),
                                      seerep_core_msgs::TimePoint(query.timeinterval.timeMax.seconds));

  std::vector<seerep_core_msgs::AabbTimeIdPair> timetree_result;
  m_timetree.query(boost::geometry::index::intersects(aabbtime), std::back_inserter(timetree_result));
  return timetree_result;
}

void CorePointCloud::addDataset(const seerep_core_msgs::DatasetIndexable& dataset)
{
  // does the default constructor of uuid generate nil uuids?
  if (dataset.header.uuidData.is_nil())
  {
    throw std::invalid_argument("invalid uuid");
  }

  addDatasetToIndices(dataset);
}

/*
 * Data without tf cannot be added to rtree
 * check if tf is now available and add data to spatial rtree
 */
void CorePointCloud::tryAddingDataWithMissingTF()
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

void CorePointCloud::addDatasetToIndices(const seerep_core_msgs::DatasetIndexable& dataset)
{
  if (m_tfOverview->canTransform(dataset.header.frameId, m_frameId, dataset.header.timestamp.seconds,
                                 dataset.header.timestamp.nanos))
  {
    m_rt.insert(
        std::make_pair(m_tfOverview->transformAABB(dataset.boundingbox, dataset.header.frameId, m_frameId,
                                                   dataset.header.timestamp.seconds, dataset.header.timestamp.nanos),
                       dataset.header.uuidData));
  }
  else
  {
    m_dataWithMissingTF.push_back(std::make_shared<seerep_core_msgs::DatasetIndexable>(dataset));
  }
  seerep_core_msgs::AabbTime aabbTime(seerep_core_msgs::TimePoint(dataset.header.timestamp.seconds),
                                      seerep_core_msgs::TimePoint(dataset.header.timestamp.seconds));
  m_timetree.insert(std::make_pair(aabbTime, dataset.header.uuidData));

  auto& labels = dataset.labels;

  for (std::string label : labels)
  {
    // check if label already exists
    std::unordered_map<std::string, std::vector<boost::uuids::uuid>>::iterator labelmapentry = m_label.find(label);
    if (labelmapentry != m_label.end())
    {
      // label already exists, add id of image to the vector
      labelmapentry->second.push_back(dataset.header.uuidData);
    }
    else
    {
      // label doesn't already exist. Create new pair of label and vector of image ids
      m_label.insert(std::make_pair(label, std::vector<boost::uuids::uuid>{ dataset.header.uuidData }));
    }
  }
}

} /* namespace seerep_core */
