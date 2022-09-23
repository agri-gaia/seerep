#ifndef SEEREP_HDF5_FB_HDF5_FB_POINT_CLOUD_H_
#define SEEREP_HDF5_FB_HDF5_FB_POINT_CLOUD_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include <seerep-hdf5-core/hdf5-core-point-cloud.h>

#include "seerep-hdf5-fb/hdf5-fb-general.h"
#include "seerep-hdf5-fb/hdf5-fb-point-cloud2-iterator.h"

// seerep-msgs
#include <seerep-msgs/point_cloud_2_generated.h>

// std
#include <any>
#include <boost/geometry.hpp>
#include <optional>

namespace seerep_hdf5_fb
{
class Hdf5FbPointCloud : public Hdf5FbGeneral
{
public:
  Hdf5FbPointCloud(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  void writePointCloud2(const std::string& uuid, const seerep::fb::PointCloud2& pointcloud2);

  std::optional<flatbuffers::grpc::Message<seerep::fb::PointCloud2>> readPointCloud2(const std::string& uuid);

  std::vector<float> loadBoundingBox(const std::string& uuid);

private:
  /*
    Note: This struct is used to store the necessary information of a point field.
    The generated seerep::fb::point struct can't be used since it doesn't have a copy
    constructor resp. assignment operator and thus can't be used in maps (see CloudInfo struct).
  */
  struct PointFieldInfo
  {
    int32_t datatype;
    uint32_t count;
  };

  struct CloudInfo
  {
    bool has_points = false;
    bool has_rgb = false;
    bool has_rgba = false;
    bool has_normals = false;
    std::map<std::string, PointFieldInfo> other_fields;
  };

  template <typename T>
  void read(const std::string& id, const std::string& fieldName, std::vector<T>& data, size_t size)
  {
    const std::string hdf5DatasetFieldPath = HDF5_GROUP_POINTCLOUD + "/" + id + "/" + fieldName;

    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
        << "reading " << fieldName << " from: " << hdf5DatasetFieldPath;

    HighFive::DataSet dataset = m_file->getDataSet(id);

    data.reserve(size);
    dataset.read(data);
  }

  template <typename T>
  void write(const std::string cloud_uuid, const std::string& field_name, const seerep::fb ::PointCloud2& cloud,
             size_t size)
  {
    const std::string id = seerep_hdf5_core::Hdf5CorePointCloud::BOUNDINGBOX + "/" + cloud_uuid + "/" + field_name;
    HighFive::DataSpace data_space(size);

    std::shared_ptr<HighFive::DataSet> dataset_ptr;
    if (!m_file->exist(id))
      dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->createDataSet<T>(id, data_space));
    else
      dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(id));

    PointCloud2ConstIterator<T> iter(cloud, field_name);
    std::vector<T> data;
    data.reserve(size);

    for (size_t i = 0; i < size; i++)
    {
      data.push_back(*iter);
      ++iter;
    }

    dataset_ptr->write(data);
  }

  template <typename T, typename... U>
  bool equalVectorLength(T const& first, U const&... rest)
  {
    return ((first.size() == rest.size()) && ...);
  }

  template <typename T>
  void writePointFieldAttributes(HighFive::AnnotateTraits<T>& object,
                                 const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::PointField>>& pointFields)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "writing point field attributes to hdf5";

    std::vector<std::string> names;
    std::vector<uint32_t> offsets, counts;
    std::vector<uint8_t> datatypes;

    for (auto pointField : pointFields)
    {
      names.push_back(pointField->name()->str());
      offsets.push_back(pointField->offset());
      datatypes.push_back(static_cast<uint8_t>(pointField->datatype()));
      counts.push_back(pointField->count());
    }

    writeAttributeToHdf5<std::vector<std::string>>(object, FIELD_NAME, names);
    writeAttributeToHdf5<std::vector<uint32_t>>(object, FIELD_OFFSET, offsets);
    writeAttributeToHdf5<std::vector<uint8_t>>(object, FIELD_DATATYPE, datatypes);
    writeAttributeToHdf5<std::vector<uint32_t>>(object, FIELD_COUNT, counts);
  }

  CloudInfo getCloudInfo(const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::PointField>>& pointFields);

  void writePoints(const std::string& uuid, const std::shared_ptr<HighFive::Group>& data_group_ptr,
                   const seerep::fb::PointCloud2& cloud);

  void writeColorsRGB(const std::string& uuid, const seerep::fb::PointCloud2& cloud);

  void writeColorsRGBA(const std::string& uuid, const seerep::fb::PointCloud2& cloud);

  void writeOtherFields(const std::string& uuid, const seerep::fb::PointCloud2& cloud,
                        const std::map<std::string, PointFieldInfo>& fields);

  void readPoints(const std::string& uuid, std::vector<std::vector<std::vector<float>>>& pointData);

  void readColorsRGB(const std::string& uuid, std::vector<std::vector<std::vector<uint8_t>>>& colorData);

  void readColorsRGBA(const std::string& uuid, std::vector<std::vector<std::vector<uint8_t>>>& colorData);

  void readOtherFields(const std::string& uuid, const std::map<std::string, PointFieldInfo>& fields,
                       std::vector<std::any> otherFieldsData);

  // pointcloud attribute keys
  inline static const std::string HEIGHT = "height";
  inline static const std::string WIDTH = "width";
  inline static const std::string ENCODING = "encoding";
  inline static const std::string IS_BIGENDIAN = "is_bigendian";
  inline static const std::string ROW_STEP = "row_step";
  inline static const std::string POINT_STEP = "point_step";
  inline static const std::string IS_DENSE = "is_dense";

  // point field attribute keys
  inline static const std::string FIELD_NAME = "field_name";
  inline static const std::string FIELD_OFFSET = "field_offset";
  inline static const std::string FIELD_DATATYPE = "field_datatype";
  inline static const std::string FIELD_COUNT = "field_count";

public:
  inline static const std::string HDF5_GROUP_POINTCLOUD = "pointclouds";
};
}  // namespace seerep_hdf5_fb

#endif /* SEEREP_HDF5_FB_HDF5_FB_POINT_CLOUD_H_ */
