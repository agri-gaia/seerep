#ifndef SEEREP_HDF5_HDF5_GENERAL_CORE_H_
#define SEEREP_HDF5_HDF5_GENERAL_CORE_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-msgs
#include <seerep_msgs/aabb.h>
#include <seerep_msgs/dataset_indexable.h>
#include <seerep_msgs/geodetic_coordinates.h>
#include <seerep_msgs/labels_with_instance_with_category.h>

// std
#include <boost/geometry.hpp>
#include <mutex>
#include <optional>
#include <type_traits>

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

namespace seerep_hdf5_core
{
class Hdf5CoreGeneral
{
public:
  Hdf5CoreGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  std::vector<std::string> getGroupDatasets(const std::string& id);
  /**
   * @brief Checks if a DataSet or DataGroup exists in the file
   * @deprecated see exists()
   * @param id The id of the DataSet or DataGroup
   */
  void checkExists(const std::string& id);

  /**
   * @brief Checks if a HDF5 path exists in the file.
   *
   * Wrapper around HighFive::File::exist() for logging purposes.
   *
   * @param path The HDF5 path to check
   * @return True if the dataset exists, false otherwise.
   */
  bool exists(const std::string& path) const;

  /**
   * @brief Reads an attribute from an HDF5 object.
   *
   * @tparam T The type of the attribute to read.
   * @tparam C The type of the HDF5 object to read from.
   * @param object The HDF5 object to read from.
   * @param attribute_name The name of the attribute to read.
   * @param path The path to the HDF5 object, only used for logging.
   * @return T The value of the attribute.
   */
  template <typename T, class C>
  T readAttributeFromHdf5(const HighFive::AnnotateTraits<C>& object, const std::string& attribute_name,
                          const std::string& path);

  /**
   * @brief Writes an attribute to an HDF5 object.
   *
   * @tparam T The type of the attribute to write.
   * @tparam C The type of the HDF5 object.
   * @param object The HDF5 object to write the attribute to.
   * @param attribute_name The name of the attribute.
   * @param attribute_val The value of the attribute.
   */
  template <typename T, class C>
  void writeAttributeToHdf5(HighFive::AnnotateTraits<C>& object, const std::string& attribute_name, T attribute_val);

  /**
   * @brief Returns a tf2 compliant frame ID.
   *
   * @param frame_id The frame ID to check.
   * @return The corresponding tf2 frame ID.
   */
  const std::string tf2_frame_id(const std::string& frame_id) const;

  /**
   * @brief Reads a tf2 compliant frame ID from an HDF5 object.
   *
   * @tparam C The type of the HDF5 object to read from.
   * @param object The HDF5 object to read from.
   * @param frame_field The name of HDF5 attribute to read the frame ID from.
   * @param id The path to the HDF5 object, only used for logging.
   * @return The frame ID as a string.
   */
  template <class C>
  std::string readFrameId(const HighFive::AnnotateTraits<C>& object, const std::string& frame_field,
                          const std::string& path);

  /**
   * @brief Writes a tf2 compliant frame ID to an HDF5 object.
   *
   * @tparam C The type of the HDF5 object to write to.
   * @param object The HDF5 object to write to.
   * @param frame_field The name of the HDF5 attribute to write the frame ID to.
   * @param frame_id The frame ID to write.
   */
  template <class C>
  void writeFrameId(HighFive::AnnotateTraits<C>& object, const std::string& frame_field, const std::string& frame_id);

  // ################
  //  AABB
  // ################
  void writeAABB(
      const std::string& datatypeGroup, const std::string& uuid,
      const boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb);

  void
  readAABB(const std::string& datatypeGroup, const std::string& uuid,
           boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb);

  bool hasAABB(const std::string& datatypeGroup, const std::string& uuid);

  // ################
  //  Header
  // ################
  /**
   * @brief Write header from a group within an hd5f file
   *
   * @tparam T Template argument of either Group or Dataset
   * @param [in] object Pointer to Dataset or Group
   * @param [out] header seerep core header struct
   */
  template <class T>
  void writeHeader(HighFive::AnnotateTraits<T>& object, seerep_core_msgs::Header header);

  /**
   * @brief Read header from a group within an hd5f file
   *
   * @tparam T Write header from hd5f file from a group
   * @param [in] id ID of Camera Intrinsics
   * @param [in] object Group or Dataset pointer
   * @param [out] header seerep core header struct
   */
  template <class T>
  void readHeader(const std::string& id, HighFive::AnnotateTraits<T>& object, seerep_core_msgs::Header& header);

  // BoundingBoxes
  /**
   * @brief read the labels and instances of a dataset of all categories and add them to the category-labels/instances map
   *
   * @param [in] datatypeGroup the data type
   * @param [in] uuid the uuid of the dataset
   * @param [in,out] labelsWithInstancesWithCategory the map from category to the instances of the category
   */
  void readBoundingBoxLabeledAndAddToLabelsWithInstancesWithCategory(
      const std::string& datatypeGroup, const std::string& uuid,
      std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>>& labelsWithInstancesWithCategory);
  void readBoundingBoxLabeled(const std::string& datatypeGroup, const std::string& uuid,
                              std::vector<std::string>& labelCategories,
                              std::vector<std::vector<std::string>>& labelsPerCategory,
                              std::vector<std::vector<float>>& labelConfidencesPerCategory,
                              std::vector<std::vector<std::vector<double>>>& boundingBoxesPerCategory,
                              std::vector<std::vector<std::string>>& instancesPerCategory, bool loadBoxes = true);

  // Labels General
  /**
   * @brief read the labels general and instances of a dataset of all categories and add them to the
   * category-labels/instances map
   *
   * @param [in] datatypeGroup the data type
   * @param [in] uuid the uuid of the dataset
   * @param [in,out] labelsWithInstancesWithCategory the map from category to the instances of the category
   */
  void readLabelsGeneralAndAddToLabelsWithInstancesWithCategory(
      const std::string& datatypeGroup, const std::string& uuid,
      std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>>& labelsWithInstancesWithCategory);
  void readLabelsGeneral(
      const std::string& datatypeGroup, const std::string& uuid, std::vector<std::string>& labelCategories,
      std::vector<std::vector<seerep_core_msgs::LabelWithInstance>>& labelsWithInstancesGeneralPerCategory);
  void writeLabelsGeneral(
      const std::string& datatypeGroup, const std::string& uuid,
      const std::vector<seerep_core_msgs::LabelsWithInstanceWithCategory>& labelsWithInstanceWithCategory);
  // ################
  //  Project
  // ################
  void writeProjectname(const std::string& projectname);
  std::string readProjectname();

  void writeProjectFrameId(const std::string& frameId);
  std::string readProjectFrameId();

  void writeVersion(const std::string& version);
  const std::optional<std::string> readVersion();

  // ################
  //  Geodetic Coordinates
  // ################
  /**
   * @brief Writes Geodetic location of the project to its HDF5 file.
   *
   * @param geocoords seerep_core_msgs Geodetic Coordinates object.
   */
  void writeGeodeticLocation(const seerep_core_msgs::GeodeticCoordinates geocoords);
  /**
   * @brief Read Geodetic location of the project from its HDF5 file.
   *
   * @return std::optional<seerep_core_msgs::GeodeticCoordinates> Optional return of Geodetic Coordinates
   */
  std::optional<seerep_core_msgs::GeodeticCoordinates> readGeodeticLocation();

  /**
   * @brief Returns a shared pointer to an HDF5 group object.
   *
   * @param group_path The path to the HDF5 group.
   * @param create If true, the group will be created if it does not already exist.
   * @return std::shared_ptr<HighFive::Group> A shared pointer to the HDF5 group object. Nullptr if the group does not
   * exist and create is false.
   */
  std::shared_ptr<HighFive::Group> getHdf5Group(const std::string& group_path, bool create = true);

  /**
   * @brief Returns a shared pointer to a HighFive::DataSet object for the given dataset path.
   *
   * @param dataset_path The path to the dataset.
   * @return std::shared_ptr<HighFive::DataSet> A shared pointer to the dataset object. Nullptr if the dataset does not exist.
   */
  std::shared_ptr<HighFive::DataSet> getHdf5DataSet(const std::string& dataset_path);

  /**
   * @brief Returns a shared pointer to a HighFive::DataSet object for the given dataset path and data space.
   *
   * A new dataset will be created if it does not already exist.
   *
   * @tparam T The data type of the dataset.
   * @param dataset_path The path to the dataset.
   * @param dataspace The data space of the dataset.
   * @return std::shared_ptr<HighFive::DataSet> A shared pointer to the HighFive dataset object.
   */
  template <class T>
  std::shared_ptr<HighFive::DataSet> getHdf5DataSet(const std::string& dataset_path,
                                                    const HighFive::DataSpace& dataspace);

  /**
   * @brief Get a shared pointer to a HighFive::Dataset object for the given path, dataspace, and properties.
   *
   * A new dataset will be created if it does not already exist.
   *
   * @tparam T The datatype of the dataset.
   * @param dataset_path The path to the dataset.
   * @param dataspace The dataspace of the dataset.
   * @param properties The properties of the dataset.
   * @return std::shared_ptr<HighFive::DataSet> A shared pointer to the HDF5 dataset.
   */
  template <class T>
  std::shared_ptr<HighFive::DataSet> getHdf5DataSet(const std::string& dataset_path,
                                                    const HighFive::DataSpace& dataspace,
                                                    const HighFive::DataSetCreateProps& properties);

  /**
   * @brief get the labels of a group/dataset matching the general prefix (label type) and return the labels matching
   * the specified type. Also extract the category from the postfix
   *
   * @param id the id of the HDF5 group
   * @param labelType the label type to be extracted (e.g. LABELS_GENERAL)
   * @param matchingLabelNames the complete name of the matching labels
   * @param matchingLabelCategory the categories of the matching labels
   */
  void getLabelCategories(std::string id, std::string labelType, std::vector<std::string>& matchingLabelCategory);

private:
  void readLabel(const std::string& id, const std::string labelType, std::vector<std::string>& labels);
  void readlabelConfidences(const std::string& id, const std::string labelType, std::vector<float>& labelConfidences);
  void readBoundingBoxes(const std::string& id, const std::string boundingBoxType,
                         std::vector<std::vector<double>>& boundingBoxes);
  void readInstances(const std::string& id, const std::string InstanceType, std::vector<std::string>& instances);

public:
  // header attribute keys
  inline static const std::string HEADER_DATATYPE = "header_datatype";
  inline static const std::string HEADER_FRAME_ID = "frame_id";
  inline static const std::string HEADER_STAMP_SECONDS = "stamp_seconds";
  inline static const std::string HEADER_STAMP_NANOS = "stamp_nanos";
  inline static const std::string HEADER_PROJECT_UUID = "header_project_uuid";
  inline static const std::string HEADER_DATA_UUID = "header_data_uuid";
  inline static const std::string HEADER_SEQ = "header_seq";

  inline static const std::string AABB_FIELD = "AABB";

  inline static const std::string PROJECTNAME = "projectname";
  inline static const std::string PROJECTFRAMEID = "projectframeid";
  inline static const std::string VERSION = "version";

  inline static const std::string GEODETICLOCATION_COORDINATESYSTEM = "geoloc_coordinatesystem";
  inline static const std::string GEODETICLOCATION_ELLIPSOID = "geoloc_ellipsoid";
  inline static const std::string GEODETICLOCATION_ALTITUDE = "geoloc_altitude";
  inline static const std::string GEODETICLOCATION_LATITUDE = "geoloc_latitude";
  inline static const std::string GEODETICLOCATION_LONGITUDE = "geoloc_longitude";

  // dataset names
  inline static const std::string RAWDATA = "rawdata";
  inline static const std::string LABELGENERAL = "labelGeneral";
  inline static const std::string LABELGENERALCONFIDENCES = "labelGeneralConfidences";
  inline static const std::string LABELGENERALINSTANCES = "labelGeneralInstances";
  inline static const std::string LABELBB = "labelBB";
  inline static const std::string LABELBBCONFIDENCES = "labelBBConfidences";
  inline static const std::string LABELBBBOXESWITHROTATION = "labelBBBoxesWithRotation";
  inline static const std::string LABELBBINSTANCES = "labelBBInstances";
  inline static const std::string POINTS = "points";

protected:
  std::shared_ptr<HighFive::File> m_file;
  std::shared_ptr<std::mutex> m_write_mtx;

  /* The logger does not change the state of the object, thus we can make it mutable, to use it in const functions */
  mutable boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

}  // namespace seerep_hdf5_core

#include "impl/hdf5_core_general.hpp"  // NOLINT

#endif /* SEEREP_HDF5_HDF5_GENERAL_CORE_H_ */
