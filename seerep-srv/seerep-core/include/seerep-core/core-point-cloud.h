#ifndef SEEREP_CORE_CORE_POINTCLOUD_H_
#define SEEREP_CORE_CORE_POINTCLOUD_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/aabb.h>
#include <seerep-msgs/query-result.h>
#include <seerep-msgs/query.h>
// seerep-hdf5-core
#include <seerep-hdf5-core/hdf5-core-point-cloud.h>
// seerep-core
#include "core-tf.h"

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
namespace seerep_core
{
/**
 * @brief This is the class handling the spatio-temporal-semantic indexing of point clouds
 *
 * point clouds can be added to the indices and spatio-temporal-semantic queries can be executed
 * to get the UUIDs of the point clouds matching the query. On startup all point clouds are loaded from the HDF5 file
 * Point clouds added during runtime must be added to the indices via the corresponding public methode
 */
class CorePointCloud
{
public:
  /**
   * @brief Constructs the object handling the indexing of point clouds
   * @param hdf5_io pointer to the HDF5 io object for point clouds
   * @param tfOverview pointer to the object handling the transformations
   * @param frameId the common frame id of the project used for the spatial index
   */
  CorePointCloud(std::shared_ptr<seerep_hdf5_core::Hdf5CorePointCloud> hdf5_io,
                 std::shared_ptr<seerep_core::CoreTf> tfOverview, std::string frameId);
  ~CorePointCloud();

  /**
   * @brief Returns a vector of UUIDs of point clouds that match the query
   * @param query the spatio-temporal-semantic query
   * @return vector of UUIDs of point clouds matching the query
   */
  std::vector<boost::uuids::uuid> getData(const seerep_core_msgs::Query& bb);

  /**
   * @brief Adds a point cloud to the spatial, temporal and semantic indices
   * @param dataset contains the relevant information for indexing
   */
  void addDataset(const seerep_core_msgs::DatasetIndexable& dataset);

private:
  /**
   * @brief fills the member variables based on the HDF5 file
   */
  void recreateDatasets();
  /**
   * @brief adds the dataset to the spatio-temporal-semantic indices
   * @param dataset the information needed to add it to the indices
   */
  void addDatasetToIndices(const seerep_core_msgs::DatasetIndexable& dataset);

  /**
   * @brief tries to add the dataset which couldn't be added to the spatial index due
   * to a missing transformtion into the frame of the index. If the transformation is now
   * available the data is added to the spatial index
   */
  void tryAddingDataWithMissingTF();
  /**
   * @brief queries the spatial index and returns a vector of bounding box / UUID pairs matching the query
   * @param query the query parameters
   * @return vector of bounding box / UUID pairs matching the query
   */
  std::vector<seerep_core_msgs::AabbIdPair> querySpatial(const seerep_core_msgs::Query& query);
  /**
   * @brief queries the temporal index and returns a vector of temporal bounding box / UUID pairs matching the query
   * @param query the query parameters
   * @return vector of temporal bounding box / UUID pairs matching the query
   */
  std::vector<seerep_core_msgs::AabbTimeIdPair> queryTemporal(const seerep_core_msgs::Query& query);
  /**
   * @brief queries the semantic index and returns the UUIDs of the point clouds matching the query
   * @param query the query parameters
   * @return set of UUIDs of the point clouds matching the query
   */
  std::set<boost::uuids::uuid> querySemantic(const seerep_core_msgs::Query& query);

  /**
   * @brief intersects the results of the spatial, temporal and semantic query and returns the UUIDs
   * of the point clouds matching the query in all three modalities
   * @param rt_result the result of the spatial query
   * @param timetree_result the result of the temporal query
   * @param semanticResult the result of the semantic query
   * @return vector of UUIDs of the point clouds matching the query in all three modalities
   */
  std::vector<boost::uuids::uuid> intersectQueryResults(std::vector<seerep_core_msgs::AabbIdPair> rt_result,
                                                        std::vector<seerep_core_msgs::AabbTimeIdPair> timetree_result,
                                                        std::set<boost::uuids::uuid> semanticResult);

  /** @brief the frame id of the spatial index*/
  std::string m_frameId;
  /** @brief shared pointer to the object handling transformations */
  std::shared_ptr<seerep_core::CoreTf> m_tfOverview;
  /** @brief shared pointer to the object handling the HDF5 io for point clouds */
  std::shared_ptr<seerep_hdf5_core::Hdf5CorePointCloud> m_hdf5_io;

  /** @brief vector of the datasets which couldn't be added to the spatial index yet due to missing tf*/
  std::vector<std::shared_ptr<seerep_core_msgs::DatasetIndexable>> m_dataWithMissingTF;
  /** @brief the spatial r-tree for the spatial index*/
  seerep_core_msgs::rtree m_rt;
  /** @brief the temporal r-tree for the temporal index*/
  seerep_core_msgs::timetree m_timetree;
  /** @brief map from label to the UUIDs of the images annotated with this label*/
  std::unordered_map<std::string, std::vector<boost::uuids::uuid>> m_label;

  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_POINTCLOUD_H_
