#ifndef SEEREP_HDF5_FB_HDF5_FB_POINT_CLOUD_H_
#define SEEREP_HDF5_FB_HDF5_FB_POINT_CLOUD_H_

// highfive
#include <highfive/H5File.hpp>

// seerep_hdf5
#include <seerep_hdf5_core/hdf5_core_point_cloud.h>

#include "hdf5_fb_general.h"
#include "hdf5_fb_point_cloud2_iterator.h"

// seerep_msgs
#include <seerep_msgs/point_cloud_2_generated.h>

// seerep_com
#include <seerep_com/point_cloud_service.grpc.fb.h>

// std
#include <boost/geometry.hpp>
#include <optional>

namespace seerep_hdf5_fb
{

class Hdf5FbPointCloud : public Hdf5FbGeneral
{
public:
  /**
   * @brief Construct a new Hdf5FbPointCloud object
   *
   * @param file shared pointer to the hdf5 file of the point cloud
   * @param write_mtx shared pointer to a mutex
   */
  Hdf5FbPointCloud(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  /**
   * @brief Writes a Flatbuffers PointCloud2 message to HDF5.
   *
   * @param uuid The UUID of the PCL.
   * @param pcl The PCL message to write.
   */
  void writePointCloud2(const std::string& uuid, const seerep::fb::PointCloud2& pcl);

  /**
   * Computes the axis-aligned bounding box (AABB) of a PCL.
   *
   * @param pcl The PCL to compute the bounding box for.
   * @return std::pair<Point, Point> The minimum and maximum corner of the AABB.
   *
   * @note We assume that the x,y and z channel are in float32.
   */
  std::pair<seerep_core_msgs::Point, seerep_core_msgs::Point> computeBoundingBox(const seerep::fb::PointCloud2& pcl);

  /**
   * @brief Writes the axis aliged bounding box (AABB) to the PCL group.
   *
   * @param uuid The UUID of the PCL.
   * @param min_corner The minimum corner of the AABB
   * @param max_corner The maximum corner of the AABB.
   */
  void writeBoundingBox(const std::string& uuid, const seerep_core_msgs::Point& min_corner,
                        const seerep_core_msgs::Point& max_corner);

  /**
   * @brief Write a BoundingBoxes flatbuffers message to hdf5
   *
   * @param id the uuid of the image data group
   * @param bbLabeledWithCategory the flatbuffers BoundingBoxes with category message
   */
  void writePointCloudBoundingBoxLabeled(
      const std::string& id,
      const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeledWithCategory>>* bbLabeledWithCategory);
  /**
   * @brief Method for reading a flatbuffers PointCloud2 message from hdf5
   *
   * @param uuid the uuid of the point cloud
   * @param withoutData omit the data field
   * @return std::optional<flatbuffers::grpc::Message<seerep::fb::PointCloud2>> returns the
   */
  std::optional<flatbuffers::grpc::Message<seerep::fb::PointCloud2>> readPointCloud2(const std::string& uuid,
                                                                                     const bool withoutData = false);

private:
  /**
   * @brief struct to store information about the PointCloud2
   *
   */
  struct CloudInfo
  {
    bool has_points = false;
    bool has_rgb = false;
    bool has_rgba = false;
    bool has_normals = false;
  };

  /**
   * @brief Writes information about point fields as attributes to hdf5
   *
   * @param object HighFive object to write to (representing a data set or data group)
   * @param pointFields flatbuffers vector of the point fields to write
   */
  template <typename T>
  void writePointFieldAttributes(HighFive::AnnotateTraits<T>& object,
                                 const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::PointField>>* pointFields)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "writing point field attributes to hdf5";

    if (pointFields)
    {
      std::vector<std::string> names;
      std::vector<uint32_t> offsets, counts;
      std::vector<uint8_t> datatypes;

      for (auto pointField : *pointFields)
      {
        names.push_back(pointField->name()->str());
        offsets.push_back(pointField->offset());
        datatypes.push_back(static_cast<uint8_t>(pointField->datatype()));
        counts.push_back(pointField->count());
      }

      writeAttributeToHdf5<std::vector<std::string>>(object, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_NAME, names);
      writeAttributeToHdf5<std::vector<uint32_t>>(object, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_OFFSET, offsets);
      writeAttributeToHdf5<std::vector<uint8_t>>(object, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_DATATYPE,
                                                 datatypes);
      writeAttributeToHdf5<std::vector<uint32_t>>(object, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_COUNT, counts);
    }
  }

  /**
   * @brief Write general attributes of the point cloud to the hdf5 group
   *
   * @param dataGroupPtr shared point to the data group
   * @param cloud the point cloud to store information from
   */
  void writeGeneralAttributes(std::shared_ptr<HighFive::Group>& dataGroupPtr, const seerep::fb::PointCloud2& cloud);

  /**
   * @brief Reads general attributes from hdf5
   *
   * @param id the uuid of the point cloud
   * @param dataGroupPtr shared pointer to the data group of the point cloud
   * @param height reference to which the height is written
   * @param width  reference to which the width is written
   * @param pointStep reference to which the pointStep is written
   * @param rowStep reference to which the rowStep is written
   * @param isBigendian reference to which isBigendian is written
   * @param isDense reference to which isDense is written
   */
  void readGeneralAttributes(const std::string& id, std::shared_ptr<HighFive::Group> dataGroupPtr, uint32_t& height,
                             uint32_t& width, uint32_t& pointStep, uint32_t& rowStep, bool& isBigendian, bool& isDense);
  /**
   * @brief Reads point field information from hdf5
   *
   * @param id uuid of the point cloud
   * @param dataGroupPtr shared pointer to the data group of the point cloud
   * @param names reference to a vector to which the names are written
   * @param offsets reference to a vector to which the offsets are written
   * @param counts reference to a vector to which the count are written
   * @param datatypes reference to a vector to which the datatypes are written
   */
  void readPointFields(const std::string& id, std::shared_ptr<HighFive::Group> dataGroupPtr,
                       std::vector<std::string>& names, std::vector<uint32_t>& offsets, std::vector<uint32_t>& counts,
                       std::vector<uint8_t>& datatypes);
  /**
   * @brief Helper method to construct a flatbuffers vector of LabelWithInstance
   *
   * @param builder reference to the used flatbuffers builder
   * @param id the uuid of the point cloud
   * @return flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<seerep::fb::LabelWithInstance>>>
   *         flatbuffers vector of LabelWithInstance
   */
  // flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<seerep::fb::LabelWithInstance>>>
  // readLabelsGeneralOffset(flatbuffers::grpc::MessageBuilder& builder, const std::string& id);

  /**
   * @brief Helper method to construct a flatbuffers vector of BoundingBoxLabeled
   *
   * @param builder reference to the used flatbuffers builder
   * @param id the uuid of the point cloud
   * @return flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeled>>>
   *         flatbuffers vector of BoundingBoxLabeled
   */
  // flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeled>>>
  // readLabelsBoundingBoxOffset(flatbuffers::grpc::MessageBuilder& builder, const std::string& id);

  /**
   * @brief Helper method to construct a flatbuffers vector of pointFields
   *
   * @param builder builder reference to the used flatbuffers builder
   * @param names vector of pointField names
   * @param offsets vector of pointField offsets
   * @param counts vector of pointField counts
   * @param datatypes vector of pointField datatypes
   * @return flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<seerep::fb::PointField>>>
   *         flatbuffers vector of pointFields
   */
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<seerep::fb::PointField>>>
  readPointFieldsOffset(flatbuffers::grpc::MessageBuilder& builder, std::vector<std::string>& names,
                        std::vector<uint32_t>& offsets, std::vector<uint32_t>& counts, std::vector<uint8_t>& datatypes);

  /**
   * @brief Get information about the fields of a point cloud from a flatbuffers PointCloud2 message
   *
   * @param cloud the point cloud to get the information about
   * @return CloudInfo extracted information in form of a CloudInfo struct
   */
  CloudInfo getCloudInfo(const seerep::fb::PointCloud2& cloud);

  /**
   * @brief Get information about the fields of a point cloud from a vector of fields
   *
   * @param fields vector of field names of a point cloud
   * @return CloudInfo extracted information in form of a CloudInfo struct
   */
  CloudInfo getCloudInfo(const std::vector<std::string>& fields);

  /**
   * @brief Get the offset of a field from a flatbuffers PointCloud2 message
   *
   * @param cloud the point cloud to get the information from
   * @param fieldName the name of the field to get the offset for
   * @return uint32_t offset of the fieldName
   */
  uint32_t getOffset(const seerep::fb::PointCloud2& cloud, const std::string& fieldName);

  std::vector<uint32_t> getOffsets(const seerep::fb::PointCloud2& cloud, const std::vector<std::string>& fields);

  /**
   * @brief Get the offset of a field from hdf5
   *
   * @param names the names of the pointFields
   * @param offsets the offsets of the pointFields
   * @param fieldName the fieldName to get the offset for
   * @param isBigendian endianness of the point cloud
   * @return uint32_t the offset of the fieldName
   */
  uint32_t getOffset(const std::vector<std::string>& names, const std::vector<uint32_t>& offsets,
                     const std::string& fieldName, bool isBigendian);

  std::vector<uint32_t> getOffsets(const std::vector<std::string>& names, const std::vector<uint32_t>& offsets,
                                   bool isBigendian, const std::vector<std::string>& fields);

  /**
   * @brief Get the offset for the special case of rgba(a)
   *
   * The rgb(a) channel is encoded in a single 32 bit integer.
   * To address an individual channel (r, g, b or a ),
   * we need to get it's offset from the start of the rgb(a) field.
   *
   * @param fieldName name of the field to get the offset for i.e "r", "g", "b" or "a"
   * @param offset the offset for the start of the rgb(a) channel
   * @param isBigendian endianness of the point cloud
   * @return uint32_t the offset for the fieldName
   */
  uint32_t rgbaOffset(const std::string& fieldName, uint32_t offset, bool isBigendian);
};
}  // namespace seerep_hdf5_fb

#endif /* SEEREP_HDF5_FB_HDF5_FB_POINT_CLOUD_H_ */
