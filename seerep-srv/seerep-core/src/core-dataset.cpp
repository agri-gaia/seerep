#include "seerep-core/core-dataset.h"

namespace seerep_core
{
CoreDataset::CoreDataset(std::shared_ptr<seerep_core::CoreTf> tfOverview,
                         std::shared_ptr<seerep_core::CoreInstances> coreInstances, const std::string& frameId)
  : m_tfOverview(tfOverview), m_coreInstances(coreInstances), m_frameId(frameId)
{
}
CoreDataset::~CoreDataset()
{
}

void CoreDataset::addDatatype(const seerep_core_msgs::Datatype& datatype,
                              std::shared_ptr<seerep_hdf5_core::Hdf5CoreDatatypeInterface> hdf5Io)
{
  DatatypeSpecifics datatypeSpecifics = { .hdf5io = hdf5Io,
                                          .dataWithMissingTF =
                                              std::vector<std::shared_ptr<seerep_core_msgs::DatasetIndexable>>(),
                                          .rt = seerep_core_msgs::rtree(),
                                          .timetree = seerep_core_msgs::timetree(),
                                          .label = std::unordered_map<std::string, std::vector<boost::uuids::uuid>>() };
  m_datatypeDatatypeSpecifcsMap.emplace(datatype, std::make_shared<DatatypeSpecifics>(datatypeSpecifics));
  recreateDatasets(datatype, hdf5Io);
}

void CoreDataset::recreateDatasets(const seerep_core_msgs::Datatype& datatype,
                                   std::shared_ptr<seerep_hdf5_core::Hdf5CoreDatatypeInterface> hdf5Io)
{
  std::vector<std::string> datasets = hdf5Io->getDatasetUuids();
  for (auto uuid : datasets)
  {
    std::cout << "found " << uuid << " in HDF5 file." << std::endl;

    try
    {
      std::optional<seerep_core_msgs::DatasetIndexable> dataset = hdf5Io->readDataset(uuid);

      if (dataset)
      {
        addDatasetToIndices(datatype, dataset.value());
      }
    }
    catch (const std::runtime_error& e)
    {
      std::cout << e.what() << std::endl;
    }
  }
}

std::vector<boost::uuids::uuid> CoreDataset::getData(const seerep_core_msgs::Query& query)
{
  std::vector<boost::uuids::uuid> result;

  // check if the data has now some tf for adding it to spatial rtree
  tryAddingDataWithMissingTF(query.header.datatype);

  auto datatypeSpecifics = m_datatypeDatatypeSpecifcsMap.at(query.header.datatype);
  // space
  std::vector<seerep_core_msgs::AabbIdPair> resultRt = querySpatial(datatypeSpecifics, query);
  // time
  std::vector<seerep_core_msgs::AabbTimeIdPair> resultTime = queryTemporal(datatypeSpecifics, query);
  // semantic
  std::set<boost::uuids::uuid> resultSemantic = querySemantic(datatypeSpecifics, query);

  return intersectQueryResults(resultRt, resultTime, resultSemantic);
}

std::vector<seerep_core_msgs::AabbIdPair>
CoreDataset::querySpatial(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics, const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::AABB aabb(seerep_core_msgs::Point(bg::get<bg::min_corner, 0>(query.boundingbox),
                                                      bg::get<bg::min_corner, 1>(query.boundingbox),
                                                      bg::get<bg::min_corner, 2>(query.boundingbox)),
                              seerep_core_msgs::Point(bg::get<bg::max_corner, 0>(query.boundingbox),
                                                      bg::get<bg::max_corner, 1>(query.boundingbox),
                                                      bg::get<bg::max_corner, 2>(query.boundingbox)));
  std::vector<seerep_core_msgs::AabbIdPair> rt_result;
  datatypeSpecifics->rt.query(boost::geometry::index::intersects(aabb), std::back_inserter(rt_result));
  return rt_result;
}

std::set<boost::uuids::uuid> CoreDataset::querySemantic(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
                                                        const seerep_core_msgs::Query& query)
{
  std::set<boost::uuids::uuid> result;
  // find the queried label in the label-imageID-map
  for (std::string labelquery : query.label)
  {
    auto labelPtr = datatypeSpecifics->label.find(labelquery);
    if (labelPtr != datatypeSpecifics->label.end())
    {
      // add all imageIDs to result set
      for (boost::uuids::uuid id : labelPtr->second)
      {
        result.insert(id);
      }
    }
  }
  return result;
}

std::vector<boost::uuids::uuid>
CoreDataset::intersectQueryResults(std::vector<seerep_core_msgs::AabbIdPair> rt_result,
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

std::vector<seerep_core_msgs::AabbTimeIdPair>
CoreDataset::queryTemporal(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics, const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::AabbTime aabbtime(seerep_core_msgs::TimePoint(query.timeinterval.timeMin.seconds),
                                      seerep_core_msgs::TimePoint(query.timeinterval.timeMax.seconds));

  std::vector<seerep_core_msgs::AabbTimeIdPair> timetree_result;
  datatypeSpecifics->timetree.query(boost::geometry::index::intersects(aabbtime), std::back_inserter(timetree_result));
  return timetree_result;
}

void CoreDataset::addDataset(const seerep_core_msgs::DatasetIndexable& dataset)
{
  /// @todo does the default constructor of uuid generate nil uuids?
  if (dataset.header.uuidData.is_nil())
  {
    throw std::invalid_argument("invalid uuid");
  }

  // check if datatype is available
  if (m_datatypeDatatypeSpecifcsMap.find(dataset.header.datatype) == m_datatypeDatatypeSpecifcsMap.end())
  {
    throw std::invalid_argument("datatype not available in seerep_core");
  }

  addDatasetToIndices(dataset.header.datatype, dataset);
}

/*
 * Data without tf cannot be added to rtree
 * check if tf is now available and add data to spatial rtree
 */
void CoreDataset::tryAddingDataWithMissingTF(const seerep_core_msgs::Datatype& datatype)
{
  auto datatypeSpecifics = m_datatypeDatatypeSpecifcsMap.at(datatype);

  for (std::vector<std::shared_ptr<seerep_core_msgs::DatasetIndexable>>::iterator it =
           datatypeSpecifics->dataWithMissingTF.begin();
       it != datatypeSpecifics->dataWithMissingTF.end();
       /*it++*/ /*<-- increment in loop itself!*/)
  {
    if (m_tfOverview->canTransform((*it)->header.frameId, m_frameId, (*it)->header.timestamp.seconds,
                                   (*it)->header.timestamp.nanos))
    {
      datatypeSpecifics->rt.insert(
          std::make_pair(m_tfOverview->transformAABB((*it)->boundingbox, (*it)->header.frameId, m_frameId,
                                                     (*it)->header.timestamp.seconds, (*it)->header.timestamp.nanos),
                         (*it)->header.uuidData));
      it = datatypeSpecifics->dataWithMissingTF.erase(it);
    }
    else
    {
      it++;
    }
  }
}

void CoreDataset::addDatasetToIndices(const seerep_core_msgs::Datatype& datatype,
                                      const seerep_core_msgs::DatasetIndexable& dataset)
{
  auto datatypeSpecifics = m_datatypeDatatypeSpecifcsMap.at(datatype);

  if (m_tfOverview->canTransform(dataset.header.frameId, m_frameId, dataset.header.timestamp.seconds,
                                 dataset.header.timestamp.nanos))
  {
    datatypeSpecifics->rt.insert(
        std::make_pair(m_tfOverview->transformAABB(dataset.boundingbox, dataset.header.frameId, m_frameId,
                                                   dataset.header.timestamp.seconds, dataset.header.timestamp.nanos),
                       dataset.header.uuidData));
  }
  else
  {
    datatypeSpecifics->dataWithMissingTF.push_back(std::make_shared<seerep_core_msgs::DatasetIndexable>(dataset));
  }

  std::cout << "secs  " << std::bitset<64>((int64_t)dataset.header.timestamp.seconds) << std::endl;
  std::cout << "shift " << std::bitset<64>((int64_t)dataset.header.timestamp.seconds << 32) << std::endl;
  std::cout << "nanos " << std::bitset<64>((int64_t)dataset.header.timestamp.nanos) << std::endl;
  std::cout
      << "combi "
      << std::bitset<64>(((int64_t)dataset.header.timestamp.seconds) << 32 | ((uint64_t)dataset.header.timestamp.nanos))
      << std::endl;

  seerep_core_msgs::AabbTime aabbTime(seerep_core_msgs::TimePoint(((int64_t)dataset.header.timestamp.seconds) << 32 |
                                                                  ((uint64_t)dataset.header.timestamp.nanos)),
                                      seerep_core_msgs::TimePoint(((int64_t)dataset.header.timestamp.seconds) << 32 |
                                                                  ((uint64_t)dataset.header.timestamp.nanos)));
  datatypeSpecifics->timetree.insert(std::make_pair(aabbTime, dataset.header.uuidData));

  auto& labels = dataset.labelsWithInstances;

  for (seerep_core_msgs::LabelWithInstance labelWithInstance : labels)
  {
    // check if label already exists
    std::unordered_map<std::string, std::vector<boost::uuids::uuid>>::iterator labelmapentry =
        datatypeSpecifics->label.find(labelWithInstance.label);
    if (labelmapentry != datatypeSpecifics->label.end())
    {
      // label already exists, add id of image to the vector
      labelmapentry->second.push_back(dataset.header.uuidData);
    }
    else
    {
      // label doesn't already exist. Create new pair of label and vector of image ids
      datatypeSpecifics->label.insert(
          std::make_pair(labelWithInstance.label, std::vector<boost::uuids::uuid>{ dataset.header.uuidData }));
    }

    // add to instance
    if (!labelWithInstance.uuidInstance.is_nil())
    {
      m_coreInstances->addDataset(labelWithInstance, dataset.header.uuidData, datatype);
    }
  }
}

void CoreDataset::addLabels(const seerep_core_msgs::Datatype& datatype, const std::vector<std::string>& labels,
                            const boost::uuids::uuid& msgUuid)
{
  auto datatypeSpecifics = m_datatypeDatatypeSpecifcsMap.at(datatype);

  for (std::string label : labels)
  {
    // check if label already exists
    std::unordered_map<std::string, std::vector<boost::uuids::uuid>>::iterator labelmapentry =
        datatypeSpecifics->label.find(label);
    if (labelmapentry != datatypeSpecifics->label.end())
    {
      // label already exists, add id of image to the vector
      labelmapentry->second.push_back(msgUuid);
    }
    else
    {
      // label doesn't already exist. Create new pair of label and vector of image ids
      datatypeSpecifics->label.insert(std::make_pair(label, std::vector<boost::uuids::uuid>{ msgUuid }));
    }
  }
}

} /* namespace seerep_core */
