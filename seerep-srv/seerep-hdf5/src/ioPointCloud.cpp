#include "seerep-hdf5/ioPointCloud.h"
#include "seerep-hdf5/ioGeneral.h"
#include "seerep-hdf5/point_cloud2_iterator.hpp"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5
{
SeerepHDF5IOPointCloud::SeerepHDF5IOPointCloud(std::shared_ptr<HighFive::File>& file,
                                               std::shared_ptr<std::mutex>& write_mtx)
  : m_file(file), m_write_mtx(write_mtx), SeerepHDF5IOGeneral(file, write_mtx)
{
}

std::map<std::string, HighFive::Group> SeerepHDF5IOPointCloud::getPointClouds()
{
  std::map<std::string, HighFive::Group> map;
  if (m_file->exist(HDF5_GROUP_POINTCLOUD))
  {
    const HighFive::Group& clouds_group = m_file->getGroup(HDF5_GROUP_POINTCLOUD);
    for (std::string& cloud_name : clouds_group.listObjectNames())
    {
      map.insert(std::pair(cloud_name, clouds_group.getGroup(cloud_name)));
    }
  }
  return map;
}

std::shared_ptr<HighFive::Group> SeerepHDF5IOPointCloud::writePointCloud2(const std::string& uuid,
                                                                          const seerep::PointCloud2& pointcloud2)
{
  std::string cloud_group_id = HDF5_GROUP_POINTCLOUD + "/" + uuid;

  std::shared_ptr<HighFive::Group> data_group_ptr;

  if (!m_file->exist(cloud_group_id))
  {
    std::cout << "data id " << cloud_group_id << " does not exist! Creat new dataset in hdf5" << std::endl;
    data_group_ptr = std::make_shared<HighFive::Group>(m_file->createGroup(cloud_group_id));
    data_group_ptr->createAttribute(HEIGHT, pointcloud2.height());
    data_group_ptr->createAttribute(WIDTH, pointcloud2.width());
    data_group_ptr->createAttribute(IS_BIGENDIAN, pointcloud2.is_bigendian());
    data_group_ptr->createAttribute(POINT_STEP, pointcloud2.point_step());
    data_group_ptr->createAttribute(ROW_STEP, pointcloud2.row_step());
    data_group_ptr->createAttribute(IS_DENSE, pointcloud2.is_dense());
  }
  else
  {
    std::cout << "data id " << cloud_group_id << " already exists!" << std::endl;
    data_group_ptr = std::make_shared<HighFive::Group>(m_file->getGroup(cloud_group_id));
    data_group_ptr->getAttribute(HEIGHT).write(pointcloud2.height());
    data_group_ptr->getAttribute(WIDTH).write(pointcloud2.width());
    data_group_ptr->getAttribute(IS_BIGENDIAN).write(pointcloud2.is_bigendian());
    data_group_ptr->getAttribute(POINT_STEP).write(pointcloud2.point_step());
    data_group_ptr->getAttribute(ROW_STEP).write(pointcloud2.row_step());
    data_group_ptr->getAttribute(IS_DENSE).write(pointcloud2.is_dense());
  }

  writePointFieldAttributes(*data_group_ptr, pointcloud2.fields());
  // TODO
  writeHeaderAttributes(*data_group_ptr, pointcloud2.header());

  std::string points_id = HDF5_GROUP_POINTCLOUD + "/" + uuid + "/points";
  HighFive::DataSpace data_space({ pointcloud2.height(), pointcloud2.width(), 3 });

  std::shared_ptr<HighFive::DataSet> points_dataset_ptr;
  if (!m_file->exist(points_id))
    points_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->createDataSet<float>(points_id, data_space));
  else
    points_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(points_id));

  std::vector<std::vector<std::vector<float>>> point_data;
  point_data.resize(pointcloud2.height());

  seerep_hdf5::PointCloud2ConstIterator<float> x_iter(pointcloud2, "x");
  seerep_hdf5::PointCloud2ConstIterator<float> y_iter(pointcloud2, "y");
  seerep_hdf5::PointCloud2ConstIterator<float> z_iter(pointcloud2, "z");

  std::array<float, 3> min, max;
  min[0] = min[1] = min[2] = std::numeric_limits<float>::max();
  max[0] = max[1] = max[2] = std::numeric_limits<float>::min();

  for (int i = 0; i < pointcloud2.height(); i++)
  {
    point_data[i].reserve(pointcloud2.width());
    for (int j = 0; j < pointcloud2.width(); j++)
    {
      const float& x = *x_iter;
      const float& y = *y_iter;
      const float& z = *z_iter;

      // compute bounding box
      if (x < min[0])
        min[0] = x;
      if (x > max[0])
        max[0] = x;

      if (y < min[1])
        min[1] = y;
      if (y > max[1])
        max[1] = y;

      if (z < min[2])
        min[2] = z;
      if (z > max[2])
        max[2] = z;

      point_data[i].push_back(std::vector{ x, y, z });

      ++x_iter, ++y_iter, ++z_iter;
    }
  }

  const std::vector boundingbox{ min[0], min[1], min[2], max[0], max[1], max[2] };
  if (!m_file->exist(cloud_group_id))
  {
    data_group_ptr->createAttribute(BOUNDINGBOX, boundingbox);
  }
  else
  {
    data_group_ptr->getAttribute(BOUNDINGBOX).write(boundingbox);
  }

  points_dataset_ptr->write(point_data);

  writeBoundingBoxLabeled(HDF5_GROUP_POINTCLOUD, uuid, pointcloud2.labels_bb());

  m_file->flush();
  return data_group_ptr;
}

// TODO read partial point cloud, e.g. only xyz without color, etc.
std::optional<seerep::PointCloud2> SeerepHDF5IOPointCloud::readPointCloud2(const std::string& id)
{
  if (!m_file->exist(id))
  {
    std::cout << "id " << id << " does not exist in file " << m_file->getName() << std::endl;
    return std::nullopt;
  }
  std::cout << "get Dataset" << std::endl;
  HighFive::DataSet data_set = m_file->getDataSet(id);

  seerep::PointCloud2 pointcloud2;

  std::cout << "read header attributes" << std::endl;
  *pointcloud2.mutable_header() = readHeaderAttributes(data_set);

  std::cout << "get attributes" << std::endl;
  uint32_t height, width, point_step, row_step;
  bool is_bigendian, is_dense;
  std::cout << "read height" << std::endl;
  data_set.getAttribute(HEIGHT).read(height);
  std::cout << "read width" << std::endl;
  data_set.getAttribute(WIDTH).read(width);
  std::cout << "read is_bigendian" << std::endl;
  data_set.getAttribute(IS_BIGENDIAN).read(is_bigendian);
  std::cout << "read point_step" << std::endl;
  data_set.getAttribute(POINT_STEP).read(point_step);
  std::cout << "read row_step" << std::endl;
  data_set.getAttribute(ROW_STEP).read(row_step);
  std::cout << "read is_dense" << std::endl;
  data_set.getAttribute(IS_DENSE).read(is_dense);

  pointcloud2.set_height(height);
  pointcloud2.set_width(width);
  pointcloud2.set_is_bigendian(is_bigendian);
  pointcloud2.set_point_step(point_step);
  pointcloud2.set_row_step(row_step);
  pointcloud2.set_is_dense(is_dense);

  std::cout << "read point field attributes" << std::endl;

  // TODO
  //*pointcloud2.mutable_fields() = readPointFieldAttributes(data_set);

  std::cout << "read Dataset" << std::endl;

  seerep_hdf5::PointCloud2Iterator<float> x_iter(pointcloud2, "x");
  seerep_hdf5::PointCloud2Iterator<float> y_iter(pointcloud2, "y");
  seerep_hdf5::PointCloud2Iterator<float> z_iter(pointcloud2, "z");

  std::vector<std::vector<std::vector<float>>> read_data;
  data_set.read(read_data);

  for (auto column : read_data)
  {
    for (auto row : column)
    {
      *x_iter = row[0];
      *y_iter = row[1];
      *z_iter = row[2];
      ++x_iter, ++y_iter, ++z_iter;
    }
  }

  return pointcloud2;
}

void SeerepHDF5IOPointCloud::writePointFieldAttributes(
    HighFive::Group& cloud_group, const google::protobuf::RepeatedPtrField<seerep::PointField> repeatedPointField)
{
  std::vector<std::string> names;
  std::vector<uint32_t> offsets, counts;
  std::vector<uint8_t> datatypes;
  for (int i = 0; i < repeatedPointField.size(); i++)
  {
    names.push_back(repeatedPointField.at(i).name());
    offsets.push_back(repeatedPointField.at(i).offset());
    datatypes.push_back(static_cast<uint8_t>(repeatedPointField.at(i).datatype()));
    counts.push_back(repeatedPointField.at(i).count());
  }

  if (!cloud_group.hasAttribute(FIELD_NAME))
    cloud_group.createAttribute(FIELD_NAME, names);
  else
    cloud_group.getAttribute(FIELD_NAME).write(names);

  if (!cloud_group.hasAttribute(FIELD_OFFSET))
    cloud_group.createAttribute(FIELD_OFFSET, offsets);
  else
    cloud_group.getAttribute(FIELD_OFFSET).write(offsets);

  if (!cloud_group.hasAttribute(FIELD_DATATYPE))
    cloud_group.createAttribute(FIELD_DATATYPE, datatypes);
  else
    cloud_group.getAttribute(FIELD_DATATYPE).write(datatypes);

  if (!cloud_group.hasAttribute(FIELD_COUNT))
    cloud_group.createAttribute(FIELD_COUNT, counts);
  else
    cloud_group.getAttribute(FIELD_COUNT).write(counts);
}

google::protobuf::RepeatedPtrField<seerep::PointField>
SeerepHDF5IOPointCloud::readPointFieldAttributes(HighFive::Group& cloud_group)
{
  google::protobuf::RepeatedPtrField<seerep::PointField> repeatedPointField;

  std::vector<std::string> names;
  std::vector<uint32_t> offsets, counts;
  std::vector<uint8_t> datatypes;

  cloud_group.getAttribute(FIELD_NAME).read(names);
  cloud_group.getAttribute(FIELD_OFFSET).read(offsets);
  cloud_group.getAttribute(FIELD_DATATYPE).read(datatypes);
  cloud_group.getAttribute(FIELD_COUNT).read(counts);

  for (int i = 0; i < names.size(); i++)
  {
    seerep::PointField point_field;

    point_field.set_name(names.at(i));
    point_field.set_offset(offsets.at(i));
    point_field.set_datatype(static_cast<seerep::PointField::Datatype>(datatypes.at(i)));
    point_field.set_count(counts.at(i));

    *repeatedPointField.Add() = point_field;
  }

  return repeatedPointField;
}

} /* namespace seerep_hdf5 */
