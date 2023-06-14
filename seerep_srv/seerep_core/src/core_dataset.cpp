#include "seerep_core/core_dataset.h"

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
    .categoryLabelDatasetsMap =
        std::unordered_map<std::string, std::unordered_map<std::string, std::vector<boost::uuids::uuid>>>(),
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
    auto instances = m_coreInstances->getDatasets(query.instances.value(), query.header.datatype);
    if (!instances.empty())
    {
      instanceResult = instances;
    }
  }

  if (!query.boundingbox && !query.timeinterval && !query.label && !query.instances && !query.dataUuids)
  {
    return getAllDatasetUuids(datatypeSpecifics);
  }

  return intersectQueryResults(resultRt, resultTime, resultSemantic, instanceResult, query.dataUuids);
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
    // a data structure for holding the result of a query to the r tree
    std::optional<std::vector<seerep_core_msgs::AabbIdPair>> rt_result = std::vector<seerep_core_msgs::AabbIdPair>();

    // retrieve axis-aligned bounding box from the requested query
    seerep_core_msgs::AABB aabb(seerep_core_msgs::Point(bg::get<bg::min_corner, 0>(query.boundingbox.value()),
                                                        bg::get<bg::min_corner, 1>(query.boundingbox.value()),
                                                        bg::get<bg::min_corner, 2>(query.boundingbox.value())),
                                seerep_core_msgs::Point(bg::get<bg::max_corner, 0>(query.boundingbox.value()),
                                                        bg::get<bg::max_corner, 1>(query.boundingbox.value()),
                                                        bg::get<bg::max_corner, 2>(query.boundingbox.value())));

    // orient aabb
    orientedBoundingBox obb = orientAABB(aabb, query.rotation);

    // perform the query on the rtree
    datatypeSpecifics->rt.query(boost::geometry::index::intersects(aabb), std::back_inserter(rt_result.value()));

    bool fullyEncapsulated;
    bool completelyDistant;
    std::vector<seerep_core_msgs::AabbIdPair>::iterator it;

    for (it = rt_result.value().begin(); it != rt_result.value().end();)
    {
      // check if query result is not fully distant to the oriented bb box provided in the query
      intersectionDegree(it->first, obb, fullyEncapsulated, completelyDistant);

      // if there is no intersection between the result and the user's request, remove it from the iterator
      if (completelyDistant)
      {
        it = rt_result.value().erase(it);
      }

      // does the user want only results fully encapsulated by the requested bb
      // and is this query result not fully encapsulated
      else if (query.encapsulated && !fullyEncapsulated)
      {
        // if yes, then delete partially encapsulated query results from the result set
        it = rt_result.value().erase(it);
      }
      else
      {
        ++it;
      }
    }

    // return the result after deletion of undesired elements has been performed
    return rt_result.value();
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
    if (query.mustHaveAllLabels)
    {
      return querySemanticWithAllTheLabels(datatypeSpecifics, query);
    }
    else
    {
      return querySemanticWithAnyOfLabels(datatypeSpecifics, query);
    }
  }

  return std::nullopt;
}

std::optional<std::set<boost::uuids::uuid>>
CoreDataset::querySemanticWithAnyOfLabels(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
                                          const seerep_core_msgs::Query& query)
{
  std::optional<std::set<boost::uuids::uuid>> result = std::set<boost::uuids::uuid>();
  // find the queried label in the label-imageID-map
  for (auto categorylabelquery : query.label.value())
  {
    auto& category = categorylabelquery.first;
    auto categoryPtr = datatypeSpecifics->categoryLabelDatasetsMap.find(category);
    if (categoryPtr != datatypeSpecifics->categoryLabelDatasetsMap.end())
    {
      for (std::string label : categorylabelquery.second)
      {
        auto labelPtr = categoryPtr->second.find(label);
        if (labelPtr != categoryPtr->second.end())
        {
          // add all imageIDs to result set
          for (boost::uuids::uuid id : labelPtr->second)
          {
            result.value().insert(id);
          }
        }
      }
    }
  }
  return result;
}

std::optional<std::set<boost::uuids::uuid>>
CoreDataset::querySemanticWithAllTheLabels(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
                                           const seerep_core_msgs::Query& query)
{
  std::set<boost::uuids::uuid> result;
  bool firstLabel = true;
  // find the queried label in the label-imageID-map
  for (auto categorylabelquery : query.label.value())
  {
    auto& category = categorylabelquery.first;
    for (std::string label : categorylabelquery.second)
    {
      auto categoryPtr = datatypeSpecifics->categoryLabelDatasetsMap.find(category);
      if (categoryPtr != datatypeSpecifics->categoryLabelDatasetsMap.end())
      {
        auto labelPtr = categoryPtr->second.find(label);
        if (labelPtr != categoryPtr->second.end())
        {
          std::set<boost::uuids::uuid> thisLabelSet;
          // vector to set
          for (boost::uuids::uuid id : labelPtr->second)
          {
            thisLabelSet.insert(id);
          }

          if (firstLabel)
          {
            result = std::move(thisLabelSet);
            firstLabel = false;
          }
          else
          {
            std::set<boost::uuids::uuid> intersection;
            std::set_intersection(result.begin(), result.end(), thisLabelSet.begin(), thisLabelSet.end(),
                                  std::inserter(intersection, intersection.begin()));
            result = std::move(intersection);
          }
        }
        else
        {
          return std::nullopt;
        }
        firstLabel = false;
        if (result.empty())
        {
          return std::nullopt;
        }
      }
      else
      {
        return std::nullopt;
      }
    }
  }
  return result;
}

std::vector<boost::uuids::uuid>
CoreDataset::intersectQueryResults(std::optional<std::vector<seerep_core_msgs::AabbIdPair>>& rt_result,
                                   std::optional<std::vector<seerep_core_msgs::AabbTimeIdPair>>& timetree_result,
                                   std::optional<std::set<boost::uuids::uuid>>& semanticResult,
                                   std::optional<std::vector<boost::uuids::uuid>>& instanceResult,
                                   const std::optional<std::vector<boost::uuids::uuid>>& dataUuids)
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

  if (dataUuids)
  {
    idsPerSingleModality.push_back(
        std::move(std::set<boost::uuids::uuid>(dataUuids.value().begin(), dataUuids.value().end())));
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

std::vector<boost::uuids::uuid>
CoreDataset::getAllDatasetUuids(std::shared_ptr<seerep_core::CoreDataset::DatatypeSpecifics> datatypeSpecifics)
{
  std::vector<seerep_core_msgs::AabbTimeIdPair> allIdsFromTimeTree = std::vector<seerep_core_msgs::AabbTimeIdPair>();
  seerep_core_msgs::AabbTime aabbtime(seerep_core_msgs::TimePoint(((int64_t)std::numeric_limits<uint32_t>::min() << 32 |
                                                                   ((uint64_t)std::numeric_limits<uint32_t>::min()))),
                                      seerep_core_msgs::TimePoint(((int64_t)std::numeric_limits<int32_t>::max()) << 32 |
                                                                  ((uint64_t)std::numeric_limits<uint32_t>::max())));

  datatypeSpecifics->timetree.query(boost::geometry::index::intersects(aabbtime),
                                    std::back_inserter(allIdsFromTimeTree));
  std::vector<boost::uuids::uuid> allIds;
  for (auto it = std::make_move_iterator(allIdsFromTimeTree.begin()),
            end = std::make_move_iterator(allIdsFromTimeTree.end());
       it != end; ++it)
  {
    allIds.push_back(std::move(it->second));
  }
  return allIds;
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

  addLabels(datatype, datatypeSpecifics, dataset.labelsWithInstancesWithCategory, dataset.header.uuidData);
}

void CoreDataset::addLabels(const seerep_core_msgs::Datatype& datatype,
                            const std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>>&
                                labelWithInstancePerCategory,
                            const boost::uuids::uuid& msgUuid)
{
  addLabels(datatype, m_datatypeDatatypeSpecificsMap.at(datatype), labelWithInstancePerCategory, msgUuid);
}

void CoreDataset::addLabels(const seerep_core_msgs::Datatype& datatype,
                            std::shared_ptr<seerep_core::CoreDataset::DatatypeSpecifics> datatypeSpecifics,
                            const std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>>&
                                labelWithInstancePerCategory,
                            const boost::uuids::uuid& msgUuid)
{
  std::vector<boost::uuids::uuid> instanceUuids;
  // loop categories
  for (auto labelWithInstanceOfCategory : labelWithInstancePerCategory)
  {
    // check if label already exists
    auto categoryMapEntry = datatypeSpecifics->categoryLabelDatasetsMap.find(labelWithInstanceOfCategory.first);
    if (categoryMapEntry == datatypeSpecifics->categoryLabelDatasetsMap.end())
    {
      auto emplaceReturn = datatypeSpecifics->categoryLabelDatasetsMap.emplace(
          labelWithInstanceOfCategory.first, std::unordered_map<std::string, std::vector<boost::uuids::uuid>>());
      if (!emplaceReturn.second)
      {
        throw std::runtime_error("could not insert label category to datatype specifics.");
      }
      categoryMapEntry = emplaceReturn.first;
    }
    // loop labels
    for (auto labelWithInstance : labelWithInstanceOfCategory.second)
    {
      // check if label already exists
      auto labelMapEntry = categoryMapEntry->second.find(labelWithInstance.label);
      if (labelMapEntry != categoryMapEntry->second.end())
      {
        // label already exists, add id of dataset to the vector
        labelMapEntry->second.push_back(msgUuid);
      }
      else
      {
        // label doesn't already exist. Create new pair of label and vector of dataset ids
        categoryMapEntry->second.emplace(labelWithInstance.label, std::vector<boost::uuids::uuid>{ msgUuid });
      }

      // add to instance
      if (!labelWithInstance.uuidInstance.is_nil())
      {
        m_coreInstances->addDataset(labelWithInstance, msgUuid, datatype);

        // collect the instance uuids of this dataset in this vector
        instanceUuids.push_back(labelWithInstance.uuidInstance);
      }
    }
  }
  // add the vector of instance uuids to the datatypespecifics
  datatypeSpecifics->datasetInstancesMap.emplace(msgUuid, instanceUuids);
}

orientedBoundingBox CoreDataset::orientAABB(const seerep_core_msgs::AABB& aabb,
                                            const std::optional<seerep_core_msgs::quaternion>& quaternion)
{
  // https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
  Eigen::Quaterniond q;

  if (quaternion)
  {
    q = Eigen::Quaterniond(quaternion.value().w, quaternion.value().x, quaternion.value().y, quaternion.value().z);
  }
  else
  {
    // if a quaternion is not provided, we set a unit quaternion
    q = Eigen::Quaterniond(1, 0, 0, 0);
  }

  float height = bg::get<bg::max_corner, 2>(aabb) - bg::get<bg::min_corner, 2>(aabb);

  // A quaternion is made using the min and max points of the BB, such that the x, y and z coordinates make up the
  // vector and the scalar is set to 0
  Eigen::Vector3d bottom_left(bg::get<bg::min_corner, 0>(aabb), bg::get<bg::min_corner, 1>(aabb), 1);
  Eigen::Vector3d top_left(bg::get<bg::min_corner, 0>(aabb), bg::get<bg::max_corner, 1>(aabb), 1);
  Eigen::Vector3d bottom_right(bg::get<bg::max_corner, 0>(aabb), bg::get<bg::min_corner, 1>(aabb), 1);
  Eigen::Vector3d top_right(bg::get<bg::max_corner, 0>(aabb), bg::get<bg::max_corner, 1>(aabb), 1);

  // rotate min point of AABB
  Eigen::Vector3d rotated_bottom_left = rotateVector(bottom_left, q);
  Eigen::Vector3d rotated_top_left = rotateVector(top_left, q);
  Eigen::Vector3d rotated_bottom_right = rotateVector(bottom_right, q);
  Eigen::Vector3d rotated_top_right = rotateVector(top_right, q);

  orientedBoundingBox obb;
  obb.bottom_left.set<0>(rotated_bottom_left.x());
  obb.bottom_left.set<1>(rotated_bottom_left.y());
  obb.top_left.set<0>(rotated_top_left.x());
  obb.top_left.set<1>(rotated_top_left.y());
  obb.top_right.set<0>(rotated_top_right.x());
  obb.top_right.set<1>(rotated_top_right.y());
  obb.bottom_right.set<0>(rotated_bottom_right.x());
  obb.bottom_right.set<1>(rotated_bottom_right.y());

  obb.height = height;  // height of the box is the same as the z-value of the box ground plane?

  return obb;
}

Eigen::Vector3d CoreDataset::rotateVector(const Eigen::Vector3d vec, const Eigen::Quaterniond quaternion)
{
  // return 2.0f * vec.dot(quaternion.vec()) * quaternion.vec() +
  //        ((quaternion.w() * quaternion.w() - vec.dot(quaternion.vec())) * vec) +
  //        2.0f * quaternion.w() * quaternion.vec().cross(vec);
  Eigen::Quaterniond vec_quaternion(0, vec.x(), vec.y(), vec.z());
  Eigen::Quaterniond rotated_vec;
  rotated_vec = quaternion * vec_quaternion * quaternion.inverse();
  return rotated_vec.vec();
}

void CoreDataset::intersectionDegree(const seerep_core_msgs::AABB& aabb, const orientedBoundingBox& obb,
                                     bool& fullEncapsulation, bool& partialEncapsulation)
{
  // convert seerep core aabb to cgal polygon
  Kernel::Point_2 points_aabb[] = {
    Kernel::Point_2(bg::get<bg::min_corner, 0>(aabb), bg::get<bg::min_corner, 1>(aabb)),
    Kernel::Point_2(bg::get<bg::max_corner, 0>(aabb), bg::get<bg::min_corner, 1>(aabb)),
    Kernel::Point_2(bg::get<bg::max_corner, 0>(aabb), bg::get<bg::max_corner, 1>(aabb)),
    Kernel::Point_2(bg::get<bg::min_corner, 0>(aabb), bg::get<bg::max_corner, 1>(aabb)),
  };
  CGAL::Polygon_2<Kernel> aabb_cgal(points_aabb, points_aabb + 4);

  // convert seerep core obb to cgal polygon
  Kernel::Point_2 points_obb[] = { Kernel::Point_2(obb.bottom_left.get<0>(), obb.bottom_left.get<1>()),
                                   Kernel::Point_2(obb.bottom_right.get<0>(), obb.bottom_right.get<1>()),
                                   Kernel::Point_2(obb.top_right.get<0>(), obb.top_right.get<1>()),
                                   Kernel::Point_2(obb.top_left.get<0>(), obb.top_left.get<1>()) };
  CGAL::Polygon_2<Kernel> obb_cgal(points_obb, points_obb + 4);

  assert(obb_cgal.is_simple());
  assert(aabb_cgal.is_simple());
  assert(obb_cgal.is_convex());
  assert(aabb_cgal.is_convex());

  // cgal polygon must be counter clockwise oriented
  if (obb_cgal.is_clockwise_oriented())
  {
    obb_cgal.reverse_orientation();
  }
  if (aabb_cgal.is_clockwise_oriented())
  {
    aabb_cgal.reverse_orientation();
  }

  assert(obb_cgal.is_counterclockwise_oriented());
  assert(aabb_cgal.is_counterclockwise_oriented());

  // intersect
  std::list<CGAL::Polygon_with_holes_2<Kernel>> intersection;
  CGAL::intersection(obb_cgal, aabb_cgal, std::back_inserter(intersection));

  // if there is no intersection, the iterator will be empty
  if (intersection.size() == 0)
  {
    partialEncapsulation = false;
    fullEncapsulation = false;

    return;
  }

  // for our use case there will only be one intersection polygon
  CGAL::Polygon_2<Kernel> intersection_polygon = intersection.front().outer_boundary();

  // are all the points of the intersection inside the obb
  bool isInside;
  // traverse the intersection
  for (CGAL::Polygon_2<Kernel>::Vertex_iterator vi = intersection_polygon.vertices_begin();
       vi != intersection_polygon.vertices_end(); ++vi)
  {
    // check if this point is inside the obb
    isInside = (obb_cgal.bounded_side(*vi) == CGAL::ON_BOUNDED_SIDE);

    // if isInside is false even once then there is only partial encapsulation
    if (!isInside)
    {
      partialEncapsulation = true;
      fullEncapsulation = false;

      return;
    }
  }

  // otherwise there is full encapsulation
  partialEncapsulation = false;
  fullEncapsulation = true;

  return;
}

seerep_core_msgs::AabbTime CoreDataset::getTimeBounds(std::vector<seerep_core_msgs::Datatype> datatypes)
{
  seerep_core_msgs::AabbTime overalltime;

  overalltime.max_corner().set<0>(std::numeric_limits<int64_t>::min());
  overalltime.min_corner().set<0>(std::numeric_limits<int64_t>::max());

  for (seerep_core_msgs::Datatype dt : datatypes)
  {
    seerep_core_msgs::AabbTime timeinterval = m_datatypeDatatypeSpecificsMap.at(dt)->timetree.bounds();

    // compare min and update if need be
    if (timeinterval.min_corner().get<0>() < overalltime.min_corner().get<0>())
    {
      overalltime.min_corner().set<0>(timeinterval.min_corner().get<0>());
    }

    // compare min and update if need be
    if (timeinterval.max_corner().get<0>() > overalltime.max_corner().get<0>())
    {
      overalltime.max_corner().set<0>(timeinterval.max_corner().get<0>());
    }
  }

  return overalltime;
}

seerep_core_msgs::AABB CoreDataset::getSpatialBounds(std::vector<seerep_core_msgs::Datatype> datatypes)
{
  seerep_core_msgs::AABB overallbb;

  // set the minimum to minimum possible for the datatype
  overallbb.max_corner().set<0>(std::numeric_limits<float>::min());
  overallbb.max_corner().set<1>(std::numeric_limits<float>::min());
  overallbb.max_corner().set<2>(std::numeric_limits<float>::min());

  // set the maximum for the maximum possible for the datatype
  overallbb.min_corner().set<0>(std::numeric_limits<float>::max());
  overallbb.min_corner().set<1>(std::numeric_limits<float>::max());
  overallbb.min_corner().set<2>(std::numeric_limits<float>::max());

  for (seerep_core_msgs::Datatype dt : datatypes)
  {
    seerep_core_msgs::AABB rtree_bounds = m_datatypeDatatypeSpecificsMap.at(dt)->rt.bounds();

    // update the min if needed for dimension 0
    if (rtree_bounds.min_corner().get<0>() < overallbb.min_corner().get<0>())
    {
      overallbb.min_corner().set<0>(rtree_bounds.min_corner().get<0>());
    }

    // update the min if needed for dimension 1
    if (rtree_bounds.min_corner().get<1>() < overallbb.min_corner().get<1>())
    {
      overallbb.min_corner().set<1>(rtree_bounds.min_corner().get<1>());
    }

    // update the min if needed for dimension 2
    if (rtree_bounds.min_corner().get<2>() < overallbb.min_corner().get<2>())
    {
      overallbb.min_corner().set<2>(rtree_bounds.min_corner().get<2>());
    }

    // update the max if needed for dimension 0
    if (rtree_bounds.max_corner().get<0>() > overallbb.max_corner().get<0>())
    {
      overallbb.max_corner().set<0>(rtree_bounds.max_corner().get<0>());
    }

    // update the max if needed for dimension 1
    if (rtree_bounds.max_corner().get<1>() > overallbb.max_corner().get<1>())
    {
      overallbb.max_corner().set<1>(rtree_bounds.max_corner().get<1>());
    }

    // update the max if needed for dimension 2
    if (rtree_bounds.max_corner().get<2>() > overallbb.max_corner().get<2>())
    {
      overallbb.max_corner().set<2>(rtree_bounds.max_corner().get<2>());
    }
  }
  return overallbb;
}

std::unordered_set<std::string> CoreDataset::getAllCategories(std::vector<seerep_core_msgs::Datatype> datatypes)
{
  std::unordered_set<std::string> categories;

  // traverse all datatypes
  for (seerep_core_msgs::Datatype dt : datatypes)
  {
    // obtain the map which holds the categories
    auto categories_map = &(m_datatypeDatatypeSpecificsMap.at(dt)->categoryLabelDatasetsMap);

    // traverse this map
    for (auto& [category, value] : *categories_map)
    {
      categories.insert(category);
    }
  }
  // return the prepared vector
  return categories;
}

std::unordered_set<std::string> CoreDataset::getAllLabels(std::vector<seerep_core_msgs::Datatype> datatypes,
                                                          std::string category)
{
  std::unordered_set<std::string> labels;

  // traverse all datatypes
  for (seerep_core_msgs::Datatype dt : datatypes)
  {
    // obtain the map pertaining to the provided category
    std::unordered_map<std::string, std::vector<boost::uuids::uuid>>* label_to_uuid_map =
        &(m_datatypeDatatypeSpecificsMap.at(dt)->categoryLabelDatasetsMap[category]);

    // traverse this map
    for (auto& [label, value] : *label_to_uuid_map)
    {
      labels.insert(label);
    }
  }
  // return the prepared vector
  return labels;
}

} /* namespace seerep_core */
