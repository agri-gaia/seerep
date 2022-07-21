#ifndef SEEREP_HDF5_PB_HDF5_PB_POINT_CLOUD_H_
#define SEEREP_HDF5_PB_HDF5_PB_POINT_CLOUD_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include <seerep-hdf5-core/hdf5-core-point-cloud.h>

#include "seerep-hdf5-pb/hdf5-pb-general.h"
#include "seerep-hdf5-pb/hdf5-pb-point-cloud2-iterator.h"

// seerep-msgs
#include <seerep-msgs/point_cloud_2.pb.h>

// std
#include <boost/geometry.hpp>
#include <optional>

namespace seerep_hdf5_pb
{
class Hdf5PbPointCloud : public Hdf5PbGeneral
{
public:
  Hdf5PbPointCloud(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  std::map<std::string, HighFive::Group> getPointClouds();

  std::shared_ptr<HighFive::Group> writePointCloud2(const std::string& uuid, const seerep::PointCloud2& pointcloud2);

  std::optional<seerep::PointCloud2> readPointCloud2(const std::string& uuid);

  std::vector<float> loadBoundingBox(const std::string& uuid);

private:
  struct CloudInfo
  {
    bool has_points = false;
    bool has_rgb = false;
    bool has_rgba = false;
    bool has_normals = false;
    std::map<std::string, seerep::PointField> other_fields;
  };

  template <typename T>
  void read(const std::string cloud_uuid, const std::string& field_name, seerep::PointCloud2& cloud, size_t size)
  {
    const std::string id = seerep_hdf5_core::Hdf5CorePointCloud::BOUNDINGBOX + "/" + cloud_uuid + "/" + field_name;
    PointCloud2Iterator<T> iter(cloud, field_name);
    HighFive::DataSet dataset = m_file->getDataSet(id);
    std::vector<T> data;
    data.reserve(size);
    dataset.read(data);

    for (auto& value : data)
    {
      *iter = value;
      ++iter;
    }
  }

  template <typename T>
  void write(const std::string cloud_uuid, const std::string& field_name, const seerep::PointCloud2& cloud, size_t size)
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

  CloudInfo getCloudInfo(const seerep::PointCloud2& cloud);

  void writePoints(const std::string& uuid, const seerep::PointCloud2& cloud);

  void writeColorsRGB(const std::string& uuid, const seerep::PointCloud2& cloud);

  void writeColorsRGBA(const std::string& uuid, const seerep::PointCloud2& cloud);

  void writeOtherFields(const std::string& uuid, const seerep::PointCloud2& cloud,
                        const std::map<std::string, seerep::PointField>& fields);

  void writePointFieldAttributes(HighFive::Group& cloud_group,
                                 const google::protobuf::RepeatedPtrField<seerep::PointField> repeatedPointField);

  void readPoints(const std::string& uuid, seerep::PointCloud2& cloud);

  void readColorsRGB(const std::string& uuid, seerep::PointCloud2& cloud);

  void readColorsRGBA(const std::string& uuid, seerep::PointCloud2& cloud);

  void readOtherFields(const std::string& uuid, seerep::PointCloud2& cloud,
                       const std::map<std::string, seerep::PointField>& fields);

  google::protobuf::RepeatedPtrField<seerep::PointField> readPointFieldAttributes(HighFive::Group& cloud_group);
};
} /* namespace seerep_hdf5_pb */

#endif /* SEEREP_HDF5_PB_HDF5_PB_POINT_CLOUD_H_ */
