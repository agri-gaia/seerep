#ifndef SEEREP_CORE_CORE_PROJECT_H_
#define SEEREP_CORE_CORE_PROJECT_H_

// miscellaneous
#include <proj.h>

#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <functional>
#include <optional>

// seerep-msgs
#include <seerep_msgs/aabb.h>
#include <seerep_msgs/camera_intrinsics.h>
#include <seerep_msgs/dataset_indexable.h>
#include <seerep_msgs/geodetic_coordinates.h>
#include <seerep_msgs/query.h>
#include <seerep_msgs/query_result_project.h>
#include <seerep_msgs/query_tf.h>

// seerep_hdf5_core
#include <seerep_hdf5_core/hdf5_core_image.h>
#include <seerep_hdf5_core/hdf5_core_instance.h>
#include <seerep_hdf5_core/hdf5_core_point.h>
#include <seerep_hdf5_core/hdf5_core_point_cloud.h>
#include <seerep_hdf5_core/hdf5_core_tf.h>

// seerep_core
#include "core_camera_intrinsics.h"
#include "core_dataset.h"
#include "core_instances.h"
#include "core_tf.h"

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
namespace seerep_core
{
/**
 * @brief This is the class acting as an inteface for a project / HDF5 file
 *
 * pointers to the objects handling a data type each are stored in this object.
 * Also the HDF5 file accessors and the corresponding mutex are stored here and
 * shared with the objects handling the different data types.
 *
 * All objects handling the data types only load data from the HDF5 file once on
 * construction. They never write data. The only exception are the instances.
 * For those the write to the HDF5 file is handled by this seerepCore
 *
 * @todo add a check if query has the adequate rights to access this project
 */
class CoreProject
{
public:
  /**
   * @brief Constructs the project object and loads the needed information from the HDF5 file
   * @param uuid the UUID of the project. HDF5 file name is UUID + ".h5" extension
   * @param path path to the folder containing the HDF5 files
   */
  CoreProject(const boost::uuids::uuid& uuid, const std::string path);
  /**
   * @brief Constructs the project object and creates a new HDF5 file
   * @param uuid the UUID of the project. HDF5 file name is UUID + ".h5" extension
   * @param path path to the folder containing the HDF5 files
   * @param projectname a plain name for the project for easier differentiation of the projects
   * @param mapFrameId the frame id of the map frame which is used to create the
   * spatial indices for this project
   * @param geodetic coordinates for the location of the site of data recording
   */
  CoreProject(const boost::uuids::uuid& uuid, const std::string path,
              const std::string projectname, const std::string mapFrameId,
              const seerep_core_msgs::GeodeticCoordinates geodeticCoords,
              const std::string version);
  ~CoreProject();

  /**
   * @brief Returns the name of the project
   * @return the name of the project as a string
   */
  std::string getName();
  /**
   * @brief Returns the map frame id used for the spatial indices
   * @return the map frame id
   */
  std::string getFrameId();
  /**
   * @brief Returns the version of seerep used to create the project
   * @return seerep version string
   */
  const std::string getVersion();

  /**
   * @brief Transfrom provided polygon to map frame from geodetic coords
   *
   * First the polygon is transformed to the projects frame and then to the
   * topocentric coordinates of the map frame. The
   * coordinates of the Spatial Reference System Identifier
   * for the project must be latitude first and longitude second.
   *
   * @param p the polygon to transform
   * @return the transformed polygon
   */
  seerep_core_msgs::Polygon2D
  transformToMapFrame(const seerep_core_msgs::Polygon2D& polygon,
                      const std::string& query_crs);

  /**
   * @brief Returns a vector of UUIDs of datasets that match the query and the
   * project UUID
   * @param query the spatio-temporal-semantic query
   * @return vector of UUIDs of images matching the query and the project UUID
   */
  seerep_core_msgs::QueryResultProject
  getDataset(seerep_core_msgs::Query& query);

  /**
   * @brief Returns a vector of UUIDs of instances that match the query and the project UUID
   * @param query the spatio-temporal-semantic query
   * @return vector of UUIDs of instances matching the query and the project UUID
   */
  seerep_core_msgs::QueryResultProject
  getInstances(seerep_core_msgs::Query& query);

  /**
   * @brief Returns the geodetic coordinates of this project
   * @return the geodetic coordinates
   */
  seerep_core_msgs::GeodeticCoordinates getGeodeticCoordinates();

  /**
   * @brief Adds a dataset to the spatial, temporal and semantic indices
   * @param dataset contains the relevant information for indexing
   */
  void addDataset(const seerep_core_msgs::DatasetIndexable& dataset);

  /**
   * @brief Adds labels to an existing dataset
   * @param datatype the targeted datatype
   * @param labelOfCategory map from category to a vector of labels to be added
   * to the dataset
   * @param msgUuid the UUID of the targeted dataset
   */
  void addLabels(
      const seerep_core_msgs::Datatype& datatype,
      const std::unordered_map<std::string, seerep_core_msgs::LabelDatumaro>&
          labelPerCategory,
      const boost::uuids::uuid& msgUuid);

  // tf
  /**
   * @brief Adds a tf to the tf buffer
   * @param tf the TransformStamped to be added to the buffer
   */
  void addTF(const geometry_msgs::TransformStamped& tf);
  /**
   * @brief Returns the queried transformation between the two given frames at
   * the given point in time
   * @param transformQuery the TF query defining the two frames and the point in time
   * @return the queried TF if it exists
   */
  std::optional<geometry_msgs::TransformStamped>
  getTF(const seerep_core_msgs::QueryTf& transformQuery);
  /**
   * @brief Returns a vector of all frames stored in the TF tree by the TF buffer
   * @return vector of frame names
   */
  std::vector<std::string> getFrames();

  /**
   * @brief reinitializes the tf buffer from hdf5
   */
  void reinitializeTFs();

  // camera intrinsics
  /**
   * @brief A function for passing camera intrinsics to the camera intrinsics
   * agnostic for for saving
   *
   * @param ci [in] Camera Intrinsics to be saved
   */
  void addCameraIntrinsics(const seerep_core_msgs::camera_intrinsics& ci);

  /**
   * @brief Get the Camera Intrinsics object
   *
   * @param camIntrinsicsUuid [in] Uuid of the Camera Intrinsics
   * @return std::optional<seerep_core_msgs::camera_intrinsics>
   */
  std::optional<seerep_core_msgs::camera_intrinsics>
  getCameraIntrinsics(boost::uuids::uuid camIntrinsicsUuid);

  /**
   * @brief Check if there exists a camera intrinsics against the provided uuid
   *
   * @param camIntrinsicsUuid UUID of Camera Intrinsics
   * @return true If Camera Intrinsics Exists
   * @return false If Camera Intrinsics Exists
   */
  bool cameraIntrinsicExists(boost::uuids::uuid camIntrinsicsUuid);

  /**
   * @brief Returns a shared pointer to the file mutex of the HDF5 file of this project
   * @return shared pointer to the HDF5 file mutex
   */
  std::shared_ptr<std::mutex> getHdf5FileMutex();
  /**
   * @brief Returns a shared pointer to the HDF5 file accessor of this project
   * @return a shared pointer to the HDF5 file accessor
   */
  std::shared_ptr<HighFive::File> getHdf5File();

  /**
   * @brief Get the the earliest and latest recorded time in a project
   *
   * @param datatypes datatypes across which this is determined
   * @return seerep_core_msgs::AabbTime Time bound object
   */
  seerep_core_msgs::AabbTime
  getTimeBounds(std::vector<seerep_core_msgs::Datatype> datatypes);

  /**
   * @brief Get the Spatial Bounds of project
   *
   * @param datatypes datatypes across which this is determined
   * @return seerep_core_msgs::AABB axis aligned bounding box of spatial information
   */
  seerep_core_msgs::AABB
  getSpatialBounds(std::vector<seerep_core_msgs::Datatype> datatypes);

  /**
   * @brief Get the All Categories saved in a project
   *
   * @param datatypes datatypes across which this is determined
   * @return std::vector<std::string> a vector of all categories
   */
  std::unordered_set<std::string>
  getAllCategories(std::vector<seerep_core_msgs::Datatype> datatypes);

  /**
   * @brief Get the all labels saved in a project
   *
   * @param datatypes datatypes across which this is determined
   * @param category the category across which all labels have to be aggregated
   * @return std::vector<std::string> vector of labels
   */
  std::unordered_set<std::string>
  getAllLabels(std::vector<seerep_core_msgs::Datatype> datatypes,
               std::string category);

private:
  /**
   * @brief Create the HDF5 file accessor and the mutex. Based on that create the
   * HDF5 IO objects handling the data IO for seerepCore
   * @param uuid the UUID of the project. HDF5 file name is UUID + ".h5" extension
   * @param path path to the folder containing the HDF5 files
   */
  void createHdf5Io(std::string path);
  /**
   * @brief Create the Objects handling the indices and load the data from the
   * HDF5 file into the indices
   */
  void recreateDatatypes();

  /**
   * @brief transform a point in-place utilizing proj's c lib transformations
   *
   * @param p the 3D point to apply the transform on
   * @param proj_tf_rawptr the raw pointer to the PJ object describing the
   *  transformation
   */
  void transformPointFwd(seerep_core_msgs::Point& p, PJ* proj_tf_rawptr);

  /** @brief the UUID of this project */
  boost::uuids::uuid m_uuid;

  /** @brief the to the folder containing the HDF5 files */
  std::string m_path;
  /** @brief the clear name of this project */
  std::string m_projectname;
  /** @brief the frame id for the spatial idices of this project */
  std::string m_frameId;
  /** @brief the geodetic coordinates of the location where the data was
   * collected in this project */
  std::optional<seerep_core_msgs::GeodeticCoordinates> m_geodeticCoordinates;
  /** @brief the version of seerep used for this project */
  std::string m_version;

  /** @brief the write mutex for the HDF5 file of this project */
  std::shared_ptr<std::mutex> m_write_mtx;
  /** @brief the accessor for the HDF5 file of this project */
  std::shared_ptr<HighFive::File> m_hdf5_file;
  /** @brief object handling the general HDF5 file IO */
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreGeneral> m_ioGeneral;
  /** @brief object handling the HDF5 file IO regarding TFs */
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreTf> m_ioTf;
  /** @brief object handling the HDF5 file IO regarding instances */
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreInstance> m_ioInstance;
  /** @brief object handling the HDF5 file IO regarding point clouds */
  std::shared_ptr<seerep_hdf5_core::Hdf5CorePointCloud> m_ioPointCloud;
  /** @brief object handling the HDF5 file IO regarding points */
  std::shared_ptr<seerep_hdf5_core::Hdf5CorePoint> m_ioPoint;
  /** @brief object handling the HDF5 file IO regarding images */
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreImage> m_ioImage;

  /** @brief object handling the Camera Intrinsics */
  std::shared_ptr<seerep_core::CoreCameraIntrinsics> m_coreCameraIntrinsics;
  /** @brief object handling the TF buffer and TF queries */
  std::shared_ptr<seerep_core::CoreTf> m_coreTfs;
  /** @brief object handling the instances */
  std::shared_ptr<seerep_core::CoreInstances> m_coreInstances;
  /** @brief object handling the dataset indices and dataset related queries */
  std::unique_ptr<seerep_core::CoreDataset> m_coreDatasets;
  /** @brief object handling the logging */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level>
      m_logger;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_PROJECT_H_
