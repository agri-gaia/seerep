#ifndef SEEREP_CORE_CORE_H_
#define SEEREP_CORE_CORE_H_

#include <curl/curl.h>
#include <jsoncpp/json/json.h>

#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <filesystem>
#include <functional>
#include <optional>
#include <regex>

// seerep-msgs
#include <seerep_msgs/dataset_indexable.h>
#include <seerep_msgs/project_info.h>
#include <seerep_msgs/query.h>
#include <seerep_msgs/query_result.h>
#include <seerep_msgs/query_tf.h>

// ros-msgs (tf)
#include <geometry_msgs/TransformStamped.h>

// seerep-core
#include "core_project.h"

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_core
{
/**
 * @brief This is the central point of access to the seerep core. All
 * interaction with the core goes through this class and is than distributed to
 * the corresponding project
 *
 * This class stores holds shared pointer to all projects. The pointers are
 * uniquely discribed by the project UUID.
 */
class Core
{
public:
  /**
   * @brief Constructs the core and loads all projects / HDF5 files from the data folder
   * @param dataFolder path to the folder containing the HDF5 files
   * @param loadHdf5Files bool flag if the HDF5 files in the data folder should
   * be read on startup (default true)
   */
  Core(std::string dataFolder, bool loadHdf5Files = true);
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
  seerep_core_msgs::QueryResult getDataset(seerep_core_msgs::Query& query);

  /**
   * @brief Returns a vector of UUIDs of instances that match the query and the project UUID
   * @param query the spatio-temporal-semantic query
   * @return vector of UUIDs of instances matching the query and the project UUID
   */
  seerep_core_msgs::QueryResult getInstances(seerep_core_msgs::Query& query);

  /**
   * @brief Adds labels to an existing dataset
   * @param datatype the targeted datatype
   * @param labelPerCategory map from category to a vector of labels to be added
   * to the dataset
   * @param msgUuid the UUID of the targeted dataset
   * @param projectuuid the UUID of the targeted project
   */
  void addLabels(
      const seerep_core_msgs::Datatype& datatype,
      const std::unordered_map<std::string, seerep_core_msgs::LabelDatumaro>&
          labelPerCategory,
      const boost::uuids::uuid& msgUuid, const boost::uuids::uuid& projectuuid);

  /**
   * @brief Adds a tf to the tf buffer
   * @param tf the TransformStamped to be added to the buffer
   * @param projectuuid the UUID of the targeted project
   */
  void addTF(const geometry_msgs::TransformStamped& tf,
             const boost::uuids::uuid& projectuuid);
  /**
   * @brief Returns the queried transformation between the two given frames at
   * the given point in time
   * @param transformQuery the query for a transformation
   * @return the queried TF if it exists
   */
  std::optional<geometry_msgs::TransformStamped>
  getTF(const seerep_core_msgs::QueryTf& transformQuery);
  /**
   * @brief Returns a vector of all frames stored in the TF tree by the TF buffer
   * @param projectuuid the UUID of the targeted project
   * @return vector of frame names
   */
  std::vector<std::string> getFrames(const boost::uuids::uuid& projectuuid);

  /**
   * @brief Recreates the TF buffer from hdf5
   * @param projectuuid the UUID of the targeted project
   */
  void reinitializeTFs(const boost::uuids::uuid& projectuuid);

  /**
   * @brief Adds a ci to the project
   * @param ci the Camera Intrinsics to be added
   * @param projectuuid the UUID of the targeted project
   */
  void addCameraIntrinsics(const seerep_core_msgs::camera_intrinsics& ci,
                           const boost::uuids::uuid& projectuuid);

  /**
   * @brief Get the Camera Intrinsics object
   *
   * @param ci_query Camera Intrinsics query which has the Camera Intrinsics and
   * Project UUIDs against which Camera Intrinsics are searched for
   * @return std::optional<seerep_core_msgs::camera_intrinsics> Option Camera
   * Intrinsics object
   */
  std::optional<seerep_core_msgs::camera_intrinsics>
  getCameraIntrinsics(const seerep_core_msgs::camera_intrinsics_query& ci_query);

  /**
   * @brief Check if Camera Intrinsics exist for the given project and camera
   * intrinsics UUID
   *
   * @param ci_query Camera Intrinsics query which has the Camera Intrinsics and
   * Project UUIDs against which Camera Intrinsics are searched for
   * @return true Camera Intrinsics exist
   * @return false Camera Intrinsics do not exist
   */
  bool cameraIntrinsicExists(
      const seerep_core_msgs::camera_intrinsics_query& ci_query);

  /**
   * @brief creates a new project / HDF5 file with the given project information
   * @param projectInfo the data needed for project creation (name, UUID, map
   * frame etc.)
   */
  void createProject(const seerep_core_msgs::ProjectInfo& projectInfo);
  /**
   * @brief returns the information of all projects
   * @return vector of project information
   */
  std::vector<seerep_core_msgs::ProjectInfo> getProjects();

  /**
   * @brief Returns a shared pointer to the file mutex of the HDF5 file of the
   * targeted project
   * @param projectuuid the UUID of the targeted project
   * @return shared pointer to the HDF5 file mutex
   */
  std::shared_ptr<std::mutex>
  getHdf5FileMutex(const boost::uuids::uuid& projectuuid);
  /**
   * @brief Returns a shared pointer to the HDF5 file accessor of the targeted project
   * @param projectuuid the UUID of the targeted project
   * @return a shared pointer to the HDF5 file accessor
   */
  std::shared_ptr<HighFive::File>
  getHdf5File(const boost::uuids::uuid& projectuuid);

  /**
   * @brief Create the project object for unindexed projects in the data folder
   * @return Vector of ProjectInfos of the newly indexed projects
   */
  std::vector<seerep_core_msgs::ProjectInfo> loadProjectsInFolder();

  /**
   * @brief removes the project from the seerep server and deletes the HDF5 file
   */
  void deleteProject(boost::uuids::uuid uuid);

  /**
   * @brief Get the minimum and maximum time interval for a dataset
   * @param uuid UUID of a dataset
   * @param datatypes A vector of datatypes for which the time bound has to be computed
   * @return seerep_core_msgs::AabbTime
   */
  seerep_core_msgs::AabbTime
  getOverallTimeInterval(boost::uuids::uuid uuid,
                         std::vector<seerep_core_msgs::Datatype> datatypes);

  /**
   * @brief Get the minimum and maximum spatial bound for a dataset
   * @param uuid UUID of a dataset
   * @param datatypes A vector of datatypes for which the spatial bound has to
   * be computed
   * @return seerep_core_msgs::AABB
   */
  seerep_core_msgs::AABB
  getOverallBound(boost::uuids::uuid uuid,
                  std::vector<seerep_core_msgs::Datatype> datatypes);

  /**
   * @brief Get the all categories saved in a project
   *
   * @param uuid of project
   * @param datatypes A vector of datatypes
   * @return std::vector<std::string> vectir if categories
   */
  std::unordered_set<std::string>
  getAllCategories(boost::uuids::uuid uuid,
                   std::vector<seerep_core_msgs::Datatype> datatypes);

  /**
   * @brief Get the all labels saved in a project
   *
   * @param uuid of project
   * @param datatypes A vector of datatypes
   * @param category the category across which all labels have to be aggregated
   * @return std::vector<std::string> uuid of project
   */
  std::unordered_set<std::string>
  getAllLabels(boost::uuids::uuid uuid,
               std::vector<seerep_core_msgs::Datatype> datatypes,
               std::string category);

private:
  /**
   * @brief Returns an iterator to the project with the given uuid. Throws an
   * error if not found
   * @param projectuuid the UUID of the project to find
   * @return an iterator to the map entry of the project object with the given
   * uuid
   */
  std::unordered_map<boost::uuids::uuid,
                     std::shared_ptr<seerep_core::CoreProject>,
                     boost::hash<boost::uuids::uuid>>::iterator
  findProject(const boost::uuids::uuid& projectuuid);

  /**
   * @brief Applies the query on all projects
   * @param query the query with the query parameters
   * @return the query result
   */
  seerep_core_msgs::QueryResult
  getDatasetFromAllProjects(seerep_core_msgs::Query& query);

  /**
   * @brief Applies the query on the specified projects
   * @param query the query with the query parameters
   * @return the query result
   */
  seerep_core_msgs::QueryResult
  getDatasetFromSpecificProjects(seerep_core_msgs::Query& query);

  /**
   * @brief Adds a dataset to the result set
   * @param dataset the dataset that should be added to the result
   * @param result the result
   */
  void addDatasetToResult(seerep_core_msgs::QueryResultProject& dataset,
                          seerep_core_msgs::QueryResult& result);

  /**
   * @brief checks the size of the queryResult and reduces it to the queried size
   *
   * @param queryResult the query result
   * @param maxNum the max number of datasets in the result
   * @return seerep_core_msgs::QueryResult the reduced result
   */
  seerep_core_msgs::QueryResult
  checkSize(const seerep_core_msgs::QueryResult& queryResult, uint maxNum);

  /**
   * @brief check if the labels are already concepts. If they aren't ask the
   * ontology for the corresponding concepts
   *
   * @param query
   */
  void checkForOntologyConcepts(seerep_core_msgs::Query& query);

  /**
   * @brief Callback function for libcurl to write the received data.
   *
   * This callback can be called multiple times for saving each chunk.
   *
   * @param contents Pointer to the received data.
   * @param size Size of each data element received. For text data, this is 1,
   * for binary the size may be larger.
   * @param nmemb The number of data elements received.
   * @param output Pointer where the received data should be saved.
   * @return size_t Total number of bytes saved
   */
  static size_t writeCallback(void* contents, size_t size, size_t nmemb,
                              std::string* output);

  /**
   * @brief get the corresponding concept to a given label from the ontology server
   *
   * @param label the label
   * @param ontologyURI the uri to the ontology server
   * @return std::string the concept
   */
  std::string translateLabelToOntologyConcept(const std::string& label,
                                              const std::string& ontologyURI);

  /**
   * @brief perform a curl using the given query at the given url
   *
   * @param query the curl statement to be used
   * @param url the target server
   * @return std::optional<std::string> the curl answer
   */
  std::optional<std::string> performCurl(std::string query, std::string url);

  /**
   * @brief Extract concepts from a JSON string
   *
   * @param json JSON string
   * @param conceptVariableName Variable name of the concept
   * @return std::optional<std::string<std::string>> The resulting concepts, if any
   */
  std::optional<std::vector<std::string>>
  extractConceptsFromJson(std::string json, std::string conceptVariableName);

  /**
   * @brief Get the Concepts for the semantic part of the query via Sparql Query object
   *
   * @param sparqlQuery the sparql query to be used
   * @param ontologyURI the URI of the ontology to be used for the query
   * @param label the resulting labels based on the query
   */
  void getConceptsViaSparqlQuery(
      const seerep_core_msgs::SparqlQuery& sparqlQuery,
      const std::string& ontologyURI,
      std::optional<std::unordered_map<std::string, std::vector<std::string>>>
          label);

  /** @brief the path to the folder containing the HDF5 files */
  std::string m_dataFolder;

  /** @brief a map from the UUID of a project the object of the project */
  std::unordered_map<boost::uuids::uuid,
                     std::shared_ptr<seerep_core::CoreProject>,
                     boost::hash<boost::uuids::uuid>>
      m_projects;
  /** @brief object handling the logging */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level>
      m_logger;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_H_
