#ifndef SEEREP_HDF5_FB_GENERAL_H_
#define SEEREP_HDF5_FB_GENERAL_H_

// highfive
#include <highfive/H5File.hpp>

// seerep_hdf5_core
#include <seerep_hdf5_core/hdf5_core_general.h>

// seerep-msgs
#include <seerep_msgs/header_generated.h>
#include <seerep_msgs/label_category_generated.h>
#include <seerep_msgs/union_map_entry_generated.h>

// grpc / flatbuffer
#include <flatbuffers/grpc.h>
#include <grpcpp/grpcpp.h>

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
typedef flatbuffers::Vector<flatbuffers::Offset<seerep::fb::Label>> LabelsFb;
typedef flatbuffers::Vector<flatbuffers::Offset<seerep::fb::LabelCategory>>
    LabelsCategoryFb;
typedef flatbuffers::Vector<flatbuffers::Offset<seerep::fb::UnionMapEntry>>
    AttributeMapsFb;

/**
 * @brief This class encompasses all read and write operations for the
 * hdf5-fb-io which can be used by different data types
 *
 */
class Hdf5FbGeneral : public virtual seerep_hdf5_core::Hdf5CoreGeneral
{
public:
  /**
   * @brief Write a flatbuffers general labels message to hdf5
   *
   * @param datatypeGroup the data type the general labels should be written to
   * e.g point cloud, image
   * @param uuid the uuid of the data group, the general labels should be written to
   * @param labels the flatbuffers general labels message
   */
  void writeLabelsFb(const std::string& datatypeGroup, const std::string& uuid,
                     const LabelsCategoryFb* labels);

protected:
  /**
   * @brief Construct a new general hdf5-fb-io object
   *
   * @param file shared pointer to the hdf5 file
   * @param write_mtx mutex to ensure thread safety
   */
  Hdf5FbGeneral(std::shared_ptr<HighFive::File>& file,
                std::shared_ptr<std::mutex>& write_mtx);

  /**
   * @brief Write a flatbuffers attribute map message to a dataset
   *
   * @param dataSetPtr shared pointer to the dataset, where the attributes
   * should be written to
   * @param attributes pointer to the flatbuffers attribute map message
   */
  void writeAttributeMap(const std::shared_ptr<HighFive::DataSet> dataSetPtr,
                         const AttributeMapsFb* attributes);

  /**
   * @brief Read a attribute map from a data set or data group and receive it as
   * a flatbuffers message
   *
   * @param object the HighFive data set ot data group to read from
   * @param builder the flatbuffers message builder
   * @return flatbuffers::Offset<AttributeMapsFb> the flatbuffers attribute map message
   */
  template <class T>
  flatbuffers::Offset<AttributeMapsFb>
  readAttributeMap(HighFive::AnnotateTraits<T>& object,
                   flatbuffers::grpc::MessageBuilder& builder);
  /**
   * @brief Write a flatbuffers header message to a data set or data group
   *
   * @param object the HighFive data set ot data group to write to
   * @param header pointer to the flatbuffers header message
   */
  template <class T>
  void writeHeaderAttributes(HighFive::AnnotateTraits<T>& object,
                             const seerep::fb::Header* header);

  /**
   * @brief Read a header from a data set or data group and receive it as a
   * flatbuffers message
   *
   * @param builder the flatbuffers message builder
   * @param object the HighFive data set ot data group to read from
   * @param uuidMsg the uuid of the message, the header belongs to (for logging
   * purposes)
   * @return flatbuffers::Offset<seerep::fb::Header> the flatbuffers header
   * message
   */
  template <class T>
  flatbuffers::Offset<seerep::fb::Header>
  readHeaderAttributes(flatbuffers::grpc::MessageBuilder& builder,
                       HighFive::AnnotateTraits<T>& object,
                       std::string uuidMsg);

  /**
   * @brief Read all general labels from a datatype into a Flatbuffers message
   *
   * @param datatypeGroup dat type the labels should be read from
   * e.g point cloud, image
   * @param uuid uuid of the HDF5 group the labels should be read from
   * @param builder a flatbuffers message builder object
   * @return std::optional<flatbuffers::Offset<LabelsCategoryFb>> all labels as
   * a Flatbuffers message, if labels are present
   */
  std::optional<flatbuffers::Offset<LabelsCategoryFb>>
  readLabels(const std::string& datatypeGroup, const std::string& uuid,
             flatbuffers::grpc::MessageBuilder& builder);
};

}  // namespace seerep_hdf5_fb

#include "impl/hdf5_fb_general.hpp"  // NOLINT

#endif /* SEEREP_HDF5_FB_GENERAL_H_ */
