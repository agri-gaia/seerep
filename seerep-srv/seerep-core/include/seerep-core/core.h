#ifndef SEEREP_CORE_CORE_H_
#define SEEREP_CORE_CORE_H_

#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <filesystem>
#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/dataset-indexable.h>
#include <seerep-msgs/project-info.h>
#include <seerep-msgs/query-result.h>
#include <seerep-msgs/query-tf.h>
#include <seerep-msgs/query.h>

// ros-msgs (tf)
#include <geometry_msgs/TransformStamped.h>

// seerep-core
#include "core-project.h"

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
namespace seerep_core
{
/**
 * @brief This is the central point of access to the seerep core. All interaction with the
 * core goes through this class and is than distributed to the corresponding project
 *
 * This class stores holds shared pointer to all projects. The pointers are uniquely discribed
 * by the project UUID.
 */
class Core
{
public:
  /**
   * @brief Constructs the core and loads all projects / HDF5 files from the data folder
   * @param datafolder path to the folder containing the HDF5 files
   */
  Core(std::string datafolder);
  ~Core();

  /**
   * @brief Adds a dataset to the spatial, temporal and semantic indices
   * @param dataset contains the relevant information for indexing
   */
  void addDataset(const seerep_core_msgs::DatasetIndexable& dataset);
  /**
   * @brief Returns a vector of UUIDs of datasets that match the query per project
   * @param query the spatio-temporal-semantic query
   * @return vector of UUIDs of datasets matching the query per project
   */
  seerep_core_msgs::QueryResult getDataset(const seerep_core_msgs::Query& query);

  /**
   * @brief Adds labels to an existing dataset
   * @param datatype the targeted datatype
   * @param labels a vector of labels to be added to the dataset
   * @param msgUuid the UUID of the targeted dataset
   * @param projectuuid the UUID of the targeted project
   */
  void addLabels(const seerep_core_msgs::Datatype& datatype, std::vector<std::string>& labels,
                 const boost::uuids::uuid& msgUuid, const boost::uuids::uuid& projectuuid);

  /**
   * @brief Adds a tf to the tf buffer
   * @param tf the TransformStamped to be added to the buffer
   * @param projectuuid the UUID of the targeted project
   */
  void addTF(const geometry_msgs::TransformStamped& tf, const boost::uuids::uuid& projectuuid);
  /**
   * @brief Returns the queried transformation between the two given frames at the given point in time
   * @param transformQuery the query for a transformation
   * @return the queried TF if it exists
   */
  std::optional<geometry_msgs::TransformStamped> getTF(const seerep_core_msgs::QueryTf& transformQuery);
  /**
   * @brief Returns a vector of all frames stored in the TF tree by the TF buffer
   * @param projectuuid the UUID of the targeted project
   * @return vector of frame names
   */
  std::vector<std::string> getFrames(const boost::uuids::uuid& projectuuid);

  /**
   * @brief creates a new project / HDF5 file with the given project information
   * @param projectInfo the data needed for project creation (name, UUID, map frame etc.)
   */
  void newProject(const seerep_core_msgs::ProjectInfo& projectInfo);
  /**
   * @brief returns the information of all projects
   * @return vector of project information
   */
  std::vector<seerep_core_msgs::ProjectInfo> getProjects();

  /**
   * @brief Returns a shared pointer to the file mutex of the HDF5 file of the targeted project
   * @param projectuuid the UUID of the targeted project
   * @return shared pointer to the HDF5 file mutex
   */
  std::shared_ptr<std::mutex> getHdf5FileMutex(const boost::uuids::uuid& projectuuid);
  /**
   * @brief Returns a shared pointer to the HDF5 file accessor of the targeted project
   * @param projectuuid the UUID of the targeted project
   * @return a shared pointer to the HDF5 file accessor
   */
  std::shared_ptr<HighFive::File> getHdf5File(const boost::uuids::uuid& projectuuid);

private:
  /**
   * @brief create the project object for each HDF5 file in the data folder
   */
  void recreateProjects();

  /** @brief the path to the folder containing the HDF5 files */
  std::string m_datafolder;

  /** @brief a map from the UUID of a project the object of the project */
  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_core::CoreProject>, boost::hash<boost::uuids::uuid>>
      m_projects;

  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_H_
