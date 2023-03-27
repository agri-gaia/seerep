#ifndef SEEREP_HDF5_FB_GENERAL_H_
#define SEEREP_HDF5_FB_GENERAL_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include <seerep-hdf5-core/hdf5-core-general.h>

// seerep-msgs
#include <seerep_msgs/boundingbox2d_labeled_generated.h>
#include <seerep_msgs/boundingbox2d_labeled_with_category_generated.h>
#include <seerep_msgs/boundingbox_labeled_generated.h>
#include <seerep_msgs/boundingbox_labeled_with_category_generated.h>
#include <seerep_msgs/header_generated.h>
#include <seerep_msgs/labels_with_instance_with_category_generated.h>
#include <seerep_msgs/union_map_entry_generated.h>

// grpc / flatbuffer
#include <grpcpp/grpcpp.h>

#include "flatbuffers/grpc.h"

// std
#include <boost/geometry.hpp>
#include <filesystem>
#include <mutex>
#include <optional>

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
namespace seerep_hdf5_fb
{
// make nested flatbuffers readable
typedef flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeled>> BoundingBoxesLabeledFb;
typedef flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeledWithCategory>>
    BoundingBoxesLabeledWithCategoryFb;
typedef flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeled>> BoundingBoxes2dLabeledFb;
typedef flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeledWithCategory>>
    BoundingBoxes2dLabeledWithCategoryFb;
typedef flatbuffers::Vector<flatbuffers::Offset<seerep::fb::LabelWithInstance>> GeneralLabelsFb;
typedef flatbuffers::Vector<flatbuffers::Offset<seerep::fb::LabelsWithInstanceWithCategory>> GeneralLabelsWithCategoryFb;
typedef flatbuffers::Vector<flatbuffers::Offset<seerep::fb::UnionMapEntry>> AttributeMapsFb;

/**
 * @brief This class encompasses all read and write operations for the hdf5-fb-io which can be used by different data types
 *
 */
class Hdf5FbGeneral : public virtual seerep_hdf5_core::Hdf5CoreGeneral
{
protected:
  /**
   * @brief Construct a new general hdf5-fb-io object
   *
   * @param file shared pointer to the hdf5 file
   * @param write_mtx mutex to ensure thread safety
   */
  Hdf5FbGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  /**
   * @brief Write a flatbuffers attribute map message to a dataset
   *
   * @param dataSetPtr shared pointer to the dataset, where the attributes should be written to
   * @param attributes pointer to the flatbuffers attribute map message
   */
  void writeAttributeMap(const std::shared_ptr<HighFive::DataSet> dataSetPtr, const AttributeMapsFb* attributes);

  /**
   * @brief Read a attribute map from a data set or data group and receive it as a flatbuffers message
   *
   * @param object the HighFive data set ot data group to read from
   * @param builder the flatbuffers message builder
   * @return flatbuffers::Offset<AttributeMapsFb> the flatbuffers attribute map message
   */
  template <class T>
  flatbuffers::Offset<AttributeMapsFb> readAttributeMap(HighFive::AnnotateTraits<T>& object,
                                                        flatbuffers::grpc::MessageBuilder& builder);
  /**
   * @brief Write a flatbuffers header message to a data set or data group
   *
   * @param object the HighFive data set ot data group to write to
   * @param header pointer to the flatbuffers header message
   */
  template <class T>
  void writeHeaderAttributes(HighFive::AnnotateTraits<T>& object, const seerep::fb::Header* header);

  /**
   * @brief Read a header from a data set or data group and receive it as a flatbuffers message
   *
   * @param builder the flatbuffers message builder
   * @param object the HighFive data set ot data group to read from
   * @param uuidMsg the uuid of the message, the header belongs to (for logging purposes)
   * @return flatbuffers::Offset<seerep::fb::Header> the flatbuffers header message
   */
  template <class T>
  flatbuffers::Offset<seerep::fb::Header> readHeaderAttributes(flatbuffers::grpc::MessageBuilder& builder,
                                                               HighFive::AnnotateTraits<T>& object,
                                                               std::string uuidMsg);

  /**
   * @brief Write a flatbuffers 3D bounding box message with category to hdf5
   *
   * @param datatypeGroup the data type the bounding box should be written to e.g point cloud, image
   * @param uuid the uuid of the data group, the bounding box should be written to
   * @param boundingBoxLabeledWithCategory the flatbuffers 3D bounding box with category message
   */
  void writeBoundingBoxLabeled(const std::string& datatypeGroup, const std::string& uuid,
                               const BoundingBoxesLabeledWithCategoryFb* boundingBoxLabeledWithCategory);
  /**
   * @brief Write a flatbuffers 3D bounding box message with category to hdf5
   *
   * @param datatypeGroup the data type the bounding box should be written to e.g point cloud, image
   * @param uuid the uuid of the data group, the bounding box should be written to
   * @param boundingboxLabeled the flatbuffers 3D bounding box message
   * @param category the category those bounding boxes are in
   */
  void writeBoundingBoxLabeled(const std::string& datatypeGroup, const std::string& uuid,
                               const BoundingBoxesLabeledFb* boundingboxLabeled, const std::string& category);
  /**
   * @brief Read the bounding boxes of all categories from hdf5 and receive it as a flatbuffers message
   *
   * @param datatypeGroup the data type the bounding box should be written to e.g image
   * @param uuid uuid of the data group, the bounding box should be written to
   * @param builder the flatbuffers message builder
   * @return flatbuffers::Offset<BoundingBoxesLabeledWithCategoryFb>
   */
  flatbuffers::Offset<BoundingBoxesLabeledWithCategoryFb>
  readBoundingBoxesLabeled(const std::string& datatypeGroup, const std::string& uuid,
                           flatbuffers::grpc::MessageBuilder& builder);
  /**
   * @brief Write a flatbuffers 2D bounding box message to hdf5
   *
   * @param datatypeGroup the data type the bounding box should be written to e.g image
   * @param uuid the uuid of the data group, the bounding box should be written to
   * @param boundingbox2dLabeled the flatbuffers 2D bounding box message
   */
  void writeBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid,
                                 const BoundingBoxes2dLabeledWithCategoryFb* boundingbox2DLabeledWithCategoryVector);

  /**
   * @brief Write a flatbuffers 2D bounding box message to hdf5
   *
   * @param datatypeGroup the data type the bounding box should be written to e.g image
   * @param uuid the uuid of the data group, the bounding box should be written to
   * @param boundingbox2dLabeled the flatbuffers 2D bounding box message
   */
  void writeBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid,
                                 const BoundingBoxes2dLabeledFb* boundingbox2DLabeled, const std::string& category);
  /**
   * @brief Read the 2D bounding boxes of all categories from hdf5 and receive it as a flatbuffers message
   *
   * @param datatypeGroup the data type the bounding box should be written to e.g image
   * @param uuid uuid of the data group, the bounding box should be written to
   * @param builder the flatbuffers message builder
   * @return flatbuffers::Offset<BoundingBoxes2dLabeledWithCategoryFb> the flatbuffers 2D bounding box message
   */
  flatbuffers::Offset<BoundingBoxes2dLabeledWithCategoryFb>
  readBoundingBoxes2DLabeled(const std::string& datatypeGroup, const std::string& uuid,
                             flatbuffers::grpc::MessageBuilder& builder);

  /**
   * @brief Write a flatbuffers general labels message to hdf5
   *
   * @param datatypeGroup the data type the general labels should be written to e.g point cloud, image
   * @param uuid the uuid of the data group, the general labels should be written to
   * @param labelsGeneral the flatbuffers general labels message
   */
  void writeLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid,
                          const GeneralLabelsWithCategoryFb* labelsGeneral);
  /**
   * @brief Read general labels (with instances and of all categories) from hdf5 and receive it as a flatbuffers message
   *
   * @param datatypeGroup the data type the general labels should be written to e.g point cloud, image
   * @param uuid the id of the data group, the general labels should be written to
   * @param builder the flatbuffers message builder
   * @return flatbuffers::Offset<GeneralLabelsWithCategoryFb> the flatbuffers general labels message (with instances and
   * of all categories)
   */
  flatbuffers::Offset<GeneralLabelsWithCategoryFb> readGeneralLabels(const std::string& datatypeGroup,
                                                                     const std::string& uuid,
                                                                     flatbuffers::grpc::MessageBuilder& builder);
};

}  // namespace seerep_hdf5_fb

#include "impl/hdf5-fb-general.hpp"  // NOLINT

#endif /* SEEREP_HDF5_FB_GENERAL_H_ */
