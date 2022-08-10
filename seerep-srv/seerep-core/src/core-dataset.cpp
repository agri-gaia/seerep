#include "seerep-core/core-dataset.h"

namespace seerep_core
{
CoreDataset::CoreDataset(std::shared_ptr<seerep_core::CoreTf> tfOverview,
                         std::shared_ptr<seerep_core::CoreInstances> coreInstances, const std::string& frameId)
  : m_frameId(frameId), m_tfOverview(tfOverview), m_coreInstances(coreInstances)
{
}
CoreDataset::~CoreDataset()
{
}

void CoreDataset::addDatatype(const seerep_core_msgs::Datatype& datatype,
                              std::shared_ptr<seerep_hdf5_core::Hdf5CoreDatatypeInterface> hdf5Io)
{
  DatatypeSpecifics datatypeSpecifics = {
    .hdf5io = hdf5Io,
    .dataWithMissingTF = std::vector<std::shared_ptr<seerep_core_msgs::DatasetIndexable>>(),
    .rt = seerep_core_msgs::rtree(),
    .timetree = seerep_core_msgs::timetree(),
    .labelDatasetsMap = std::unordered_map<std::string, std::vector<boost::uuids::uuid>>(),
    .datasetInstancesMap =
        std::unordered_map<boost::uuids::uuid, std::vector<boost::uuids::uuid>, boost::hash<boost::uuids::uuid>>()
  };
  m_datatypeDatatypeSpecificsMap.emplace(datatype, std::make_shared<DatatypeSpecifics>(datatypeSpecifics));
  recreateDatasets(datatype, hdf5Io);
}

void CoreDataset::recreateDatasets(const seerep_core_msgs::Datatype& datatype,
                                   std::shared_ptr<seerep_hdf5_core::Hdf5CoreDatatypeInterface> hdf5Io)
{
  std::vector<std::string> datasets = hdf5Io->getDatasetUuids();
  for (auto uuid : datasets)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "found " << uuid << " in HDF5 file.";

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
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    }
  }
}

std::vector<boost::uuids::uuid> CoreDataset::getData(const seerep_core_msgs::Query& query)
{
  std::vector<boost::uuids::uuid> result;

  // check if the data has now some tf for adding it to spatial rtree
  tryAddingDataWithMissingTF(query.header.datatype);

  auto datatypeSpecifics = m_datatypeDatatypeSpecificsMap.at(query.header.datatype);
  // space
  auto resultRt = querySpatial(datatypeSpecifics, query);
  // time
  auto resultTime = queryTemporal(datatypeSpecifics, query);
  // semantic
  auto resultSemantic = querySemantic(datatypeSpecifics, query);

  // instances
  std::optional<std::vector<boost::uuids::uuid>> instanceResult;
  if (query.instances)
  {
    instanceResult.value() = m_coreInstances->getDatasets(query.instances.value(), query.header.datatype);
  }

  return intersectQueryResults(resultRt, resultTime, resultSemantic, instanceResult);
}

std::vector<boost::uuids::uuid> CoreDataset::getInstances(const seerep_core_msgs::Query& query)
{
  std::vector<boost::uuids::uuid> result;
  if (query.header.datatype == seerep_core_msgs::Datatype::Unknown)
  {
    auto queryLocal = query;
    for (auto& datatypeSpecifics : m_datatypeDatatypeSpecificsMap)
    {
      queryLocal.header.datatype = datatypeSpecifics.first;
      auto datasets = getData(queryLocal);
      getUuidsFromMap(datatypeSpecifics.second->datasetInstancesMap, datasets, result);
    }
  }
  else
  {
    auto datasets = getData(query);
    getUuidsFromMap(m_datatypeDatatypeSpecificsMap.at(query.header.datatype)->datasetInstancesMap, datasets, result);
  }
  return result;
}

void CoreDataset::getUuidsFromMap(std::unordered_map<boost::uuids::uuid, std::vector<boost::uuids::uuid>,
                                                     boost::hash<boost::uuids::uuid>>& datasetInstancesMap,
                                  std::vector<boost::uuids::uuid>& datasets, std::vector<boost::uuids::uuid>& result)
{
  std::set<boost::uuids::uuid> instances;
  for (auto dataset : datasets)
  {
    // find the dataset in the dataset-instances-map
    auto resultPtr = datasetInstancesMap.find(dataset);

    if (resultPtr != datasetInstancesMap.end())
    {
      // add instances of dataset to instances set
      std::copy(resultPtr->second.begin(), resultPtr->second.end(), std::inserter(instances, instances.end()));
    }
  }

  std::copy(instances.begin(), instances.end(), std::back_inserter(result));
}

std::optional<std::vector<seerep_core_msgs::AabbIdPair>>
CoreDataset::querySpatial(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics, const seerep_core_msgs::Query& query)
{
  if (query.boundingbox)
  {
    std::optional<std::vector<seerep_core_msgs::AabbIdPair>> rt_result = std::vector<seerep_core_msgs::AabbIdPair>();
    // axis-aligned bounding box
    seerep_core_msgs::AABB aabb(seerep_core_msgs::Point(bg::get<bg::min_corner, 0>(query.boundingbox.value()),
                                                        bg::get<bg::min_corner, 1>(query.boundingbox.value()),
                                                        bg::get<bg::min_corner, 2>(query.boundingbox.value())),
                                seerep_core_msgs::Point(bg::get<bg::max_corner, 0>(query.boundingbox.value()),
                                                        bg::get<bg::max_corner, 1>(query.boundingbox.value()),
                                                        bg::get<bg::max_corner, 2>(query.boundingbox.value())));

    datatypeSpecifics->rt.query(boost::geometry::index::intersects(aabb), std::back_inserter(rt_result.value()));
    return rt_result;
  }
  else
  {
    return std::nullopt;
  }
}

std::optional<std::vector<seerep_core_msgs::AabbTimeIdPair>>
CoreDataset::queryTemporal(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics, const seerep_core_msgs::Query& query)
{
  if (query.timeinterval)
  {
    std::optional<std::vector<seerep_core_msgs::AabbTimeIdPair>> timetree_result =
        std::vector<seerep_core_msgs::AabbTimeIdPair>();
    seerep_core_msgs::AabbTime aabbtime(
        seerep_core_msgs::TimePoint(((int64_t)query.timeinterval.value().timeMin.seconds) << 32 |
                                    ((uint64_t)query.timeinterval.value().timeMin.nanos)),
        seerep_core_msgs::TimePoint(((int64_t)query.timeinterval.value().timeMax.seconds) << 32 |
                                    ((uint64_t)query.timeinterval.value().timeMax.nanos)));

    datatypeSpecifics->timetree.query(boost::geometry::index::intersects(aabbtime),
                                      std::back_inserter(timetree_result.value()));
    return timetree_result;
  }
  else
  {
    return std::nullopt;
  }
}

std::optional<std::set<boost::uuids::uuid>>
CoreDataset::querySemantic(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics, const seerep_core_msgs::Query& query)
{
  if (query.label)
  {
    std::optional<std::set<boost::uuids::uuid>> result = std::set<boost::uuids::uuid>();
    // find the queried label in the label-imageID-map
    for (std::string labelquery : query.label.value())
    {
      auto labelPtr = datatypeSpecifics->labelDatasetsMap.find(labelquery);
      if (labelPtr != datatypeSpecifics->labelDatasetsMap.end())
      {
        // add all imageIDs to result set
        for (boost::uuids::uuid id : labelPtr->second)
        {
          result.value().insert(id);
        }
      }
    }
    return result;
  }

  return std::nullopt;
}

std::vector<boost::uuids::uuid>
CoreDataset::intersectQueryResults(std::optional<std::vector<seerep_core_msgs::AabbIdPair>>& rt_result,
                                   std::optional<std::vector<seerep_core_msgs::AabbTimeIdPair>>& timetree_result,
                                   std::optional<std::set<boost::uuids::uuid>>& semanticResult,
                                   std::optional<std::vector<boost::uuids::uuid>>& instanceResult)
{
  std::vector<std::set<boost::uuids::uuid>> idsPerSingleModality;

  if (rt_result)
  {
    std::set<boost::uuids::uuid> idsSpatial;
    for (auto it = std::make_move_iterator(rt_result.value().begin()),
              end = std::make_move_iterator(rt_result.value().end());
         it != end; ++it)
    {
      idsSpatial.insert(std::move(it->second));
    }
    idsPerSingleModality.push_back(std::move(idsSpatial));
  }

  if (timetree_result)
  {
    std::set<boost::uuids::uuid> idsTemporal;
    for (auto it = std::make_move_iterator(timetree_result.value().begin()),
              end = std::make_move_iterator(timetree_result.value().end());
         it != end; ++it)
    {
      idsTemporal.insert(std::move(it->second));
    }
    idsPerSingleModality.push_back(std::move(idsTemporal));
  }

  if (semanticResult)
  {
    idsPerSingleModality.push_back(std::move(semanticResult.value()));
  }

  if (instanceResult)
  {
    idsPerSingleModality.push_back(
        std::move(std::set<boost::uuids::uuid>(instanceResult.value().begin(), instanceResult.value().end())));
  }

  return intersectVectorOfSets(idsPerSingleModality);
}

std::vector<boost::uuids::uuid>
CoreDataset::intersectVectorOfSets(std::vector<std::set<boost::uuids::uuid>>& vectorOfSets)
{
  if (vectorOfSets.size() > 1)
  {
    // intersect two sets until the vector has only one set remaining

    std::set<boost::uuids::uuid> intersectionResult;
    // intersect the last and the second-last set
    std::set_intersection(vectorOfSets.at(vectorOfSets.size() - 1).begin(),
                          vectorOfSets.at(vectorOfSets.size() - 1).end(),
                          vectorOfSets.at(vectorOfSets.size() - 2).begin(),
                          vectorOfSets.at(vectorOfSets.size() - 2).end(),
                          std::inserter(intersectionResult, intersectionResult.begin()));
    // pop the two intersected sets
    vectorOfSets.pop_back();
    vectorOfSets.pop_back();

    // add the intersection result to the vector
    vectorOfSets.push_back(std::move(intersectionResult));
    // recursive call until all sets are intersected
    return intersectVectorOfSets(vectorOfSets);
  }
  else if (vectorOfSets.size() == 1)
  {
    // return the result as soon as all sets are intersected
    return std::vector(vectorOfSets.at(0).begin(), vectorOfSets.at(0).end());
  }

  // return empty vector if input vector is empty
  return std::vector<boost::uuids::uuid>();
}

void CoreDataset::addDataset(const seerep_core_msgs::DatasetIndexable& dataset)
{
  /// @todo does the default constructor of uuid generate nil uuids?
  if (dataset.header.uuidData.is_nil())
  {
    throw std::invalid_argument("invalid uuid");
  }

  // check if datatype is available
  if (m_datatypeDatatypeSpecificsMap.find(dataset.header.datatype) == m_datatypeDatatypeSpecificsMap.end())
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
  auto datatypeSpecifics = m_datatypeDatatypeSpecificsMap.at(datatype);

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
  auto datatypeSpecifics = m_datatypeDatatypeSpecificsMap.at(datatype);

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

  seerep_core_msgs::AabbTime aabbTime(seerep_core_msgs::TimePoint(((int64_t)dataset.header.timestamp.seconds) << 32 |
                                                                  ((uint64_t)dataset.header.timestamp.nanos)),
                                      seerep_core_msgs::TimePoint(((int64_t)dataset.header.timestamp.seconds) << 32 |
                                                                  ((uint64_t)dataset.header.timestamp.nanos)));
  datatypeSpecifics->timetree.insert(std::make_pair(aabbTime, dataset.header.uuidData));

  auto& labels = dataset.labelsWithInstances;

  std::vector<boost::uuids::uuid> instanceUuids;
  for (seerep_core_msgs::LabelWithInstance labelWithInstance : labels)
  {
    // check if label already exists
    std::unordered_map<std::string, std::vector<boost::uuids::uuid>>::iterator labelmapentry =
        datatypeSpecifics->labelDatasetsMap.find(labelWithInstance.label);
    if (labelmapentry != datatypeSpecifics->labelDatasetsMap.end())
    {
      // label already exists, add id of image to the vector
      labelmapentry->second.push_back(dataset.header.uuidData);
    }
    else
    {
      // label doesn't already exist. Create new pair of label and vector of image ids
      datatypeSpecifics->labelDatasetsMap.insert(
          std::make_pair(labelWithInstance.label, std::vector<boost::uuids::uuid>{ dataset.header.uuidData }));
    }

    // add to instance
    if (!labelWithInstance.uuidInstance.is_nil())
    {
      m_coreInstances->addDataset(labelWithInstance, dataset.header.uuidData, datatype);

      // collect the instance uuids of this dataset in this vector
      instanceUuids.push_back(labelWithInstance.uuidInstance);
    }
  }
  // add the vector of instance uuids to the datatypespecifics
  datatypeSpecifics->datasetInstancesMap.emplace(dataset.header.uuidData, instanceUuids);
}

void CoreDataset::addLabels(const seerep_core_msgs::Datatype& datatype, const std::vector<std::string>& labels,
                            const boost::uuids::uuid& msgUuid)
{
  auto datatypeSpecifics = m_datatypeDatatypeSpecificsMap.at(datatype);

  for (std::string label : labels)
  {
    // check if label already exists
    std::unordered_map<std::string, std::vector<boost::uuids::uuid>>::iterator labelmapentry =
        datatypeSpecifics->labelDatasetsMap.find(label);
    if (labelmapentry != datatypeSpecifics->labelDatasetsMap.end())
    {
      // label already exists, add id of image to the vector
      labelmapentry->second.push_back(msgUuid);
    }
    else
    {
      // label doesn't already exist. Create new pair of label and vector of image ids
      datatypeSpecifics->labelDatasetsMap.insert(std::make_pair(label, std::vector<boost::uuids::uuid>{ msgUuid }));
    }
  }
}

} /* namespace seerep_core */
