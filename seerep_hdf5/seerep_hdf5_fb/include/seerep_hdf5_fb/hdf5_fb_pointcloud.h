#ifndef SEEREP_HDF5_FB_HDF5_FB_POINT_CLOUD_H_
#define SEEREP_HDF5_FB_HDF5_FB_POINT_CLOUD_H_

// highfive
#include <highfive/H5File.hpp>

// seerep_hdf5
#include <seerep_hdf5_core/hdf5_core_point_cloud.h>

#include "hdf5_fb_general.h"

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
  Hdf5FbPointCloud(std::shared_ptr<HighFive::File>& file,
                   std::shared_ptr<std::mutex>& write_mtx);

  /**
   * @brief Writes a Flatbuffers PointCloud2 message to HDF5.
   *
   * @param uuid The UUID of the PCL.
   * @param pcl The PCL message to write.
   */
  void writePointCloud2(const std::string& uuid,
                        const seerep::fb::PointCloud2& pcl);

  /**
   * Computes the axis-aligned bounding box (AABB) of a PCL.
   *
   * @param pcl The PCL to compute the bounding box for.
   * @return std::pair<Point, Point> The minimum and maximum corner of the AABB.
   *
   * @note We assume that the x,y and z channel are in float32.
   */
  std::pair<seerep_core_msgs::Point, seerep_core_msgs::Point>
  computeBoundingBox(const seerep::fb::PointCloud2& pcl);

  /**
   * @brief Writes the axis aliged bounding box (AABB) to the PCL group.
   *
   * @param uuid The UUID of the PCL.
   * @param min_corner The minimum corner of the AABB
   * @param max_corner The maximum corner of the AABB.
   */
  void writeBoundingBox(const std::string& uuid,
                        const seerep_core_msgs::Point& min_corner,
                        const seerep_core_msgs::Point& max_corner);

  /**
   * @brief Method for reading a flatbuffers PointCloud2 message from hdf5
   *
   * @param uuid the uuid of the point cloud
   * @param withoutData omit the data field
   * @return std::optional<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>
   * returns the
   */
  std::optional<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>
  readPointCloud2(const std::string& uuid, const bool withoutData = false);

private:
  /**
   * @brief Writes the channel description and the layout of the binary payload to HDF5.
   *
   * @tparam T HDF5 object type (group or dataset).
   * @param object The object to annotate.
   * @param pointFields The vector of point fields to write.
   */
  template <typename T>
  void writePointFieldAttributes(
      HighFive::AnnotateTraits<T>& object,
      const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::PointField>>*
          pointFields);

  /**
   * @brief Write general attributes of the point cloud to the hdf5 group
   *
   * @param dataGroupPtr shared point to the data group
   * @param cloud the point cloud to store information from
   */
  void writeGeneralAttributes(std::shared_ptr<HighFive::Group>& dataGroupPtr,
                              const seerep::fb::PointCloud2& cloud);

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
  void readGeneralAttributes(const std::string& id,
                             std::shared_ptr<HighFive::Group> dataGroupPtr,
                             uint32_t& height, uint32_t& width,
                             uint32_t& pointStep, uint32_t& rowStep,
                             bool& isBigendian, bool& isDense);
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
  void readPointFields(const std::string& id,
                       std::shared_ptr<HighFive::Group> dataGroupPtr,
                       std::vector<std::string>& names,
                       std::vector<uint32_t>& offsets,
                       std::vector<uint32_t>& counts,
                       std::vector<uint8_t>& datatypes);

  /**
   * @brief Helper method to construct a flatbuffers vector of pointFields
   *
   * @param builder builder reference to the used flatbuffers builder
   * @param names vector of pointField names
   * @param offsets vector of pointField offsets
   * @param counts vector of pointField counts
   * @param datatypes vector of pointField datatypes
   * @return
   * flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<seerep::fb::PointField>>>
   *         flatbuffers vector of pointFields
   */
  flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<seerep::fb::PointField>>>
  readPointFieldsOffset(flatbuffers::grpc::MessageBuilder& builder,
                        std::vector<std::string>& names,
                        std::vector<uint32_t>& offsets,
                        std::vector<uint32_t>& counts,
                        std::vector<uint8_t>& datatypes);

  /**
   * @brief Returns the offset of a channel relative to the start of a point.
   *
   * @param pcl The pcl message.
   * @param channel_name The name of the channel.
   * @return The offset of the channel.
   * @throws runtime_error when the requested channel is not present in the pcl.
   */
  uint32_t getChannelOffset(const seerep::fb::PointCloud2& pcl,
                            const std::string& channel_name) const;

  /**
   * @brief Returns the offset for a rgb(a) channel.
   *
   * The rgb(a) color information of a point is packed into a single uint32.
   * We therefore need to add an addtion offset to the start of the rgb(a) channel.
   *
   * @param channel_name Name of the color channel to retrieve the offset for.
   * @param base_offset The offset to the start of the rgb(a) channel.
   * @param is_big_endian The endianness of the point cloud.
   * @return The offset of the specified channel.
   * @throws runtime_error when the channel_name is not "r", "g", "b" or "a".
   */
  uint32_t getRgbaOffset(const std::string& channel_name, uint32_t base_offset,
                         bool is_big_endian) const;
};

}  // namespace seerep_hdf5_fb

#include "impl/hdf5_fb_pointcloud.hpp"

#endif /* SEEREP_HDF5_FB_HDF5_FB_POINT_CLOUD_H_ */
