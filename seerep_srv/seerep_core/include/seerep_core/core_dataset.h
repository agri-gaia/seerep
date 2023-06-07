#ifndef SEEREP_CORE_CORE_DATASET_H_
#define SEEREP_CORE_CORE_DATASET_H_

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cstdint>
#include <functional>
#include <optional>
#include <unordered_set>

// seerep-msgs
#include <seerep_msgs/aabb.h>
#include <seerep_msgs/datatype.h>
#include <seerep_msgs/query.h>

// seerep_hdf5_core
#include <seerep_hdf5_core/hdf5_core_datatype_interface.h>

// seerep_core
#include "core_instances.h"
#include "core_tf.h"

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

struct orientedBoundingBox
{
  seerep_core_msgs::Point2D bottom_right;
  seerep_core_msgs::Point2D top_right;
  seerep_core_msgs::Point2D bottom_left;
  seerep_core_msgs::Point2D top_left;
  float height;
};

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;

namespace seerep_core
{
/**
 * @brief This is the class handling the spatio-temporal-semantic indexing of a dataset
 *
 * a dataset can be added to the indices and spatio-temporal-semantic queries can be executed
 * to get the UUIDs of the datasets matching the query. On startup all datasets are loaded from the HDF5 file
 * Datasets added during runtime must be added to the indices via the corresponding public method
 */
class CoreDataset
{
private:
  /** @brief a struct for the variables that are needed for each datatype */
  struct DatatypeSpecifics
  {
    /** @brief shared pointer to the object handling the HDF5 io for the datatype */
    std::shared_ptr<seerep_hdf5_core::Hdf5CoreDatatypeInterface> hdf5io;
    /** @brief vector of the datasets which couldn't be added to the spatial index yet due to missing tf*/
    std::vector<std::shared_ptr<seerep_core_msgs::DatasetIndexable>> dataWithMissingTF =
        std::vector<std::shared_ptr<seerep_core_msgs::DatasetIndexable>>();
    /** @brief the spatial r-tree for the spatial index*/
    seerep_core_msgs::rtree rt = seerep_core_msgs::rtree();
    /** @brief the temporal r-tree for the temporal index*/
    seerep_core_msgs::timetree timetree = seerep_core_msgs::timetree();
    /** @brief map from the category of labels to the map from label to the UUIDs of the datasets annotated with this label*/
    std::unordered_map<std::string, std::unordered_map<std::string, std::vector<boost::uuids::uuid>>>
        categoryLabelDatasetsMap =
            std::unordered_map<std::string, std::unordered_map<std::string, std::vector<boost::uuids::uuid>>>();
    /** @brief map from the UUID of the dataset a vector of UUID of instances the dataset is showing */
    std::unordered_map<boost::uuids::uuid, std::vector<boost::uuids::uuid>, boost::hash<boost::uuids::uuid>>
        datasetInstancesMap =
            std::unordered_map<boost::uuids::uuid, std::vector<boost::uuids::uuid>, boost::hash<boost::uuids::uuid>>();
  };

public:
  /**
   * @brief Constructs the object handling the indexing of datasets
   * @param tfOverview pointer to the object handling the transformations
   * @param coreInstances pointer to the object handling the instances
   * @param frameId the common frame id of the project used for the spatial index
   */
  CoreDataset(std::shared_ptr<seerep_core::CoreTf> tfOverview,
              std::shared_ptr<seerep_core::CoreInstances> coreInstances, const std::string& frameId);
  ~CoreDataset();

  /**
   * @brief Adds a datatype to the core, loads the needed information from the HDF5 file and creates the indices
   * @param datatype the datatype to be added
   * @param hdf5Io pointer to the object handling HDF5 io for this datatype
   */
  void addDatatype(const seerep_core_msgs::Datatype& datatype,
                   std::shared_ptr<seerep_hdf5_core::Hdf5CoreDatatypeInterface> hdf5Io);

  /**
   * @brief Returns a vector of UUIDs of datasets that match the query
   * @param query the spatio-temporal-semantic query
   * @return vector of UUIDs of datasets matching the query
   */
  std::vector<boost::uuids::uuid> getData(const seerep_core_msgs::Query& query);

  /**
   * @brief Returns a vector of UUIDs of instances that match the query
   * @param query the spatio-temporal-semantic query
   * @return vector of UUIDs of instances matching the query
   */
  std::vector<boost::uuids::uuid> getInstances(const seerep_core_msgs::Query& query);

  /**
   * @brief Adds an image to the spatial, temporal and semantic indices
   * @param dataset contains the relevant information for indexing
   */
  void addDataset(const seerep_core_msgs::DatasetIndexable& dataset);
  /**
   * @brief Adds labels to an existing dataset
   * @param datatype the datatype to consider
   * @param labelWithInstancePerCategory map from category to a vector of labels to be added to the dataset
   * @param msgUuid the UUID of the targeted dataset
   */
  void addLabels(const seerep_core_msgs::Datatype& datatype,
                 const std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>>&
                     labelWithInstancePerCategory,
                 const boost::uuids::uuid& msgUuid);
  /**
   * @brief Adds labels to an existing dataset
   * @param datatype the datatype to consider
   * @param datatypeSpecifics the datatypeSpecifics
   * @param labelWithInstancePerCategory map from category to a vector of labels to be added to the dataset
   * @param msgUuid the UUID of the targeted dataset
   */
  void addLabels(const seerep_core_msgs::Datatype& datatype,
                 std::shared_ptr<seerep_core::CoreDataset::DatatypeSpecifics> datatypeSpecifics,
                 const std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>>&
                     labelWithInstancePerCategory,
                 const boost::uuids::uuid& msgUuid);

private:
  orientedBoundingBox orientAABB(const seerep_core_msgs::AABB& aabb,
                                 const std::optional<seerep_core_msgs::quaternion>& quaternion);

  Eigen::Vector3d rotateVector(const Eigen::Vector3d vec, const Eigen::Quaterniond quaternion);

  void intersectionDegree(const seerep_core_msgs::AABB& aabb, const orientedBoundingBox& obb, bool& fullEncapsulation,
                          bool& partialEncapsulation);

  /**
   * @brief fills the member variables based on the HDF5 file
   * @param datatype the datatype to consider
   * @param hdf5io the HDF5io object handling the HDF5 io
   */
  void recreateDatasets(const seerep_core_msgs::Datatype& datatype,
                        std::shared_ptr<seerep_hdf5_core::Hdf5CoreDatatypeInterface> hdf5Io);
  /**
   * @brief adds the dataset to the spatio-temporal-semantic indices
   * @param datatype the datatype to consider
   * @param dataset the information needed to add it to the indices
   */
  void addDatasetToIndices(const seerep_core_msgs::Datatype& datatype,
                           const seerep_core_msgs::DatasetIndexable& dataset);

  /**
   * @brief tries to add the dataset which couldn't be added to the spatial index due
   * to a missing transformtion into the frame of the index. If the transformation is now
   * available the data is added to the spatial index
   * @param datatype the datatype to consider
   */
  void tryAddingDataWithMissingTF(const seerep_core_msgs::Datatype& datatype);

  void getUuidsFromMap(std::unordered_map<boost::uuids::uuid, std::vector<boost::uuids::uuid>,
                                          boost::hash<boost::uuids::uuid>>& datasetInstancesMap,
                       std::vector<boost::uuids::uuid>& datasets, std::vector<boost::uuids::uuid>& result);
  /**
   * @brief queries the spatial index and returns a vector of bounding box / UUID pairs matching the query
   * @param datatypeSpecifics the datatype specific information to be used in the query
   * @param query the query parameters
   * @return vector of bounding box / UUID pairs matching the query
   */
  std::optional<std::vector<seerep_core_msgs::AabbIdPair>>
  querySpatial(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics, const seerep_core_msgs::Query& query);
  /**
   * @brief queries the temporal index and returns a vector of temporal bounding box / UUID pairs matching the query
   * @param datatypeSpecifics the datatype specific information to be used in the query
   * @param query the query parameters
   * @return vector of temporal bounding box / UUID pairs matching the query
   */
  std::optional<std::vector<seerep_core_msgs::AabbTimeIdPair>>
  queryTemporal(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics, const seerep_core_msgs::Query& query);
  /**
   * @brief queries the semantic index and returns the UUIDs of the images matching the query
   * @param datatypeSpecifics the datatype specific information to be used in the query
   * @param query the query parameters
   * @return set of UUIDs of the images matching the query
   */
  std::optional<std::set<boost::uuids::uuid>> querySemantic(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
                                                            const seerep_core_msgs::Query& query);

  /**
   * @brief queries the semantic index where the dataset contains any of the labels
   * @param datatypeSpecifics the datatype specific information to be used in the query
   * @param query the query parameters
   * @return set of UUIDs of the images matching the query
   */
  std::optional<std::set<boost::uuids::uuid>>
  querySemanticWithAnyOfLabels(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
                               const seerep_core_msgs::Query& query);

  /**
   * @brief queries the semantic index where the dataset contains all of the labels
   * @param datatypeSpecifics the datatype specific information to be used in the query
   * @param query the query parameters
   * @return set of UUIDs of the images matching the query
   */
  std::optional<std::set<boost::uuids::uuid>>
  querySemanticWithAllTheLabels(std::shared_ptr<DatatypeSpecifics> datatypeSpecifics,
                                const seerep_core_msgs::Query& query);

  /**
   * @brief intersects the results of the spatial, temporal and semantic query and returns the UUIDs
   * of the images matching the query in all three modalities
   * @param rt_result the result of the spatial query
   * @param timetree_result the result of the temporal query
   * @param semanticResult the result of the semantic query
   * @param instanceResult the result of the instance based query
   * @param dataUuids the uuids of the dataset specified in the query
   * @return vector of UUIDs of the images matching the query in all three modalities
   */
  std::vector<boost::uuids::uuid>
  intersectQueryResults(std::optional<std::vector<seerep_core_msgs::AabbIdPair>>& rt_result,
                        std::optional<std::vector<seerep_core_msgs::AabbTimeIdPair>>& timetree_result,
                        std::optional<std::set<boost::uuids::uuid>>& semanticResult,
                        std::optional<std::vector<boost::uuids::uuid>>& instanceResult,
                        const std::optional<std::vector<boost::uuids::uuid>>& dataUuids);

  /**
   * @brief intersects a vector of sets pairwise recursively until one intersection set remains
   * @param vectorOfSets the vector of sets to be intersected
   * @return vector of UUIDs of the intersection result
   */
  std::vector<boost::uuids::uuid> intersectVectorOfSets(std::vector<std::set<boost::uuids::uuid>>& vectorOfSets);

  /**
   * @brief return the UUIDs of all stored datasets. Uses the timeTree, because all datasets are in there
   * @param datatypeSpecifics the datatype specifics of the targeted data type
   * @return vector of UUIDs of all data sets
   */
  std::vector<boost::uuids::uuid>
  getAllDatasetUuids(std::shared_ptr<seerep_core::CoreDataset::DatatypeSpecifics> datatypeSpecifics);

  /** @brief the frame id of the spatial index*/
  std::string m_frameId;
  /** @brief shared pointer to the object handling transformations */
  std::shared_ptr<seerep_core::CoreTf> m_tfOverview;
  /** @brief shared pointer to the object handling the instances */
  std::shared_ptr<seerep_core::CoreInstances> m_coreInstances;
  /** @brief map from the datatype to the struct with the specific objects for that datatype*/
  std::unordered_map<seerep_core_msgs::Datatype, std::shared_ptr<DatatypeSpecifics>> m_datatypeDatatypeSpecificsMap;
  /** @brief object handling the logging */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_DATASET_H_
