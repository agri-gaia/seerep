#include "seerep-hdf5/pointcloud-io.h"
#include "seerep-hdf5/point_cloud2_iterator.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5
{
PointCloudIO::PointCloudIO(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : GeneralIO(file, write_mtx)
{
}

std::map<std::string, HighFive::Group> PointCloudIO::getPointClouds()
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

std::shared_ptr<HighFive::Group> PointCloudIO::writePointCloud2(const std::string& uuid,
                                                                const seerep::PointCloud2& pointcloud2)
{
  std::string cloud_group_id = HDF5_GROUP_POINTCLOUD + "/" + uuid;

  std::shared_ptr<HighFive::Group> data_group_ptr;

  if (!m_file->exist(cloud_group_id))
  {
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
    data_group_ptr = std::make_shared<HighFive::Group>(m_file->getGroup(cloud_group_id));
    data_group_ptr->getAttribute(HEIGHT).write(pointcloud2.height());
    data_group_ptr->getAttribute(WIDTH).write(pointcloud2.width());
    data_group_ptr->getAttribute(IS_BIGENDIAN).write(pointcloud2.is_bigendian());
    data_group_ptr->getAttribute(POINT_STEP).write(pointcloud2.point_step());
    data_group_ptr->getAttribute(ROW_STEP).write(pointcloud2.row_step());
    data_group_ptr->getAttribute(IS_DENSE).write(pointcloud2.is_dense());
  }

  writePointFieldAttributes(*data_group_ptr, pointcloud2.fields());
  writeHeaderAttributes(*data_group_ptr, pointcloud2.header());

  CloudInfo info = getCloudInfo(pointcloud2);

  if (info.has_points)
    writePoints(uuid, pointcloud2);

  m_file->flush();
  return data_group_ptr;
}

void PointCloudIO::writePoints(const std::string& uuid, const seerep::PointCloud2& cloud)
{
  std::string points_id = HDF5_GROUP_POINTCLOUD + "/" + uuid + "/points";
  HighFive::DataSpace data_space({ cloud.height(), cloud.width(), 3 });

  std::shared_ptr<HighFive::DataSet> points_dataset_ptr;
  if (!m_file->exist(points_id))
    points_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->createDataSet<float>(points_id, data_space));
  else
    points_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(points_id));

  std::vector<std::vector<std::vector<float>>> point_data;
  point_data.resize(cloud.height());

  seerep_hdf5::PointCloud2ConstIterator<float> x_iter(cloud, "x");
  seerep_hdf5::PointCloud2ConstIterator<float> y_iter(cloud, "y");
  seerep_hdf5::PointCloud2ConstIterator<float> z_iter(cloud, "z");

  std::array<float, 3> min, max;
  min[0] = min[1] = min[2] = std::numeric_limits<float>::max();
  max[0] = max[1] = max[2] = std::numeric_limits<float>::min();

  for (int i = 0; i < cloud.height(); i++)
  {
    point_data[i].reserve(cloud.width());
    for (int j = 0; j < cloud.width(); j++)
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

  // write bounding box as attribute to dataset
  const std::vector boundingbox{ min[0], min[1], min[2], max[0], max[1], max[2] };
  if (!points_dataset_ptr->hasAttribute(BOUNDINGBOX))
    points_dataset_ptr->createAttribute(BOUNDINGBOX, boundingbox);
  else
    points_dataset_ptr->getAttribute(BOUNDINGBOX).write(boundingbox);

  // write data to dataset
  points_dataset_ptr->write(point_data);

  // TODO what is this?
  // writeBoundingBoxLabeled(HDF5_GROUP_POINTCLOUD, uuid, pointcloud2.labels_bb());
}

void PointCloudIO::writeColorsRGB(const std::string& uuid, const seerep::PointCloud2& cloud)
{
  // TODO
}

void PointCloudIO::writeColorsRGBA(const std::string& uuid, const seerep::PointCloud2& cloud)
{
  // TODO
}

PointCloudIO::CloudInfo PointCloudIO::getCloudInfo(const seerep::PointCloud2& cloud)
{
  CloudInfo info;
  for (auto& field : cloud.fields())
  {
    if (field.name() == "xyz")
      info.has_points = true;
    else if (field.name() == "rgb")
      info.has_rgb = true;
    else if (field.name() == "rgba")
      info.has_rgba = true;
    else if (field.name().find("normal") == 0)
      info.has_normals = true;
    else
      info.other_fields[field.name()] = field;
  }
  return info;
}

// TODO read partial point cloud, e.g. only xyz without color, etc.
std::optional<seerep::PointCloud2> PointCloudIO::readPointCloud2(const std::string& uuid)
{
  if (!m_file->exist(uuid))
  {
    return std::nullopt;
  }
  HighFive::Group cloud_group = m_file->getGroup(uuid);

  seerep::PointCloud2 pointcloud2;

  *pointcloud2.mutable_header() = readHeaderAttributes(cloud_group);

  uint32_t height, width, point_step, row_step;
  bool is_bigendian, is_dense;
  cloud_group.getAttribute(HEIGHT).read(height);
  cloud_group.getAttribute(WIDTH).read(width);
  cloud_group.getAttribute(IS_BIGENDIAN).read(is_bigendian);
  cloud_group.getAttribute(POINT_STEP).read(point_step);
  cloud_group.getAttribute(ROW_STEP).read(row_step);
  cloud_group.getAttribute(IS_DENSE).read(is_dense);

  pointcloud2.set_height(height);
  pointcloud2.set_width(width);
  pointcloud2.set_is_bigendian(is_bigendian);
  pointcloud2.set_point_step(point_step);
  pointcloud2.set_row_step(row_step);
  pointcloud2.set_is_dense(is_dense);

  *pointcloud2.mutable_fields() = readPointFieldAttributes(cloud_group);

  // TODO build header and Point Fields

  CloudInfo info = getCloudInfo(pointcloud2);

  if (info.has_points)
    readPoints(uuid, pointcloud2);

  if (info.has_rgb)
    readColorsRGB(uuid, pointcloud2);

  if (info.has_rgba)
    readColorsRGBA(uuid, pointcloud2);

  return pointcloud2;
}

void PointCloudIO::readPoints(const std::string& uuid, seerep::PointCloud2& cloud)
{
  seerep_hdf5::PointCloud2Iterator<float> x_iter(cloud, "x");
  seerep_hdf5::PointCloud2Iterator<float> y_iter(cloud, "y");
  seerep_hdf5::PointCloud2Iterator<float> z_iter(cloud, "z");

  std::vector<std::vector<std::vector<float>>> point_data;
  std::string points_id = HDF5_GROUP_POINTCLOUD + "/" + uuid + "/points";

  HighFive::DataSet points_dataset = m_file->getDataSet(points_id);

  points_dataset.read(point_data);

  for (auto column : point_data)
  {
    for (auto row : column)
    {
      *x_iter = row[0];
      *y_iter = row[1];
      *z_iter = row[2];
      ++x_iter, ++y_iter, ++z_iter;
    }
  }
}

void PointCloudIO::readColorsRGB(const std::string& uuid, seerep::PointCloud2& cloud)
{
  seerep_hdf5::PointCloud2Iterator<uint8_t> r_iter(cloud, "r");
  seerep_hdf5::PointCloud2Iterator<uint8_t> g_iter(cloud, "g");
  seerep_hdf5::PointCloud2Iterator<uint8_t> b_iter(cloud, "b");

  std::vector<std::vector<std::vector<uint8_t>>> color_data;
  std::string colors_id = HDF5_GROUP_POINTCLOUD + "/" + uuid + "/colors";

  HighFive::DataSet colors_dataset = m_file->getDataSet(colors_id);

  colors_dataset.read(color_data);

  for (auto column : color_data)
  {
    for (auto row : column)
    {
      *r_iter = row[0];
      *g_iter = row[1];
      *b_iter = row[2];
      ++r_iter, ++g_iter, ++b_iter;
    }
  }
}

void PointCloudIO::readColorsRGBA(const std::string& uuid, seerep::PointCloud2& cloud)
{
  seerep_hdf5::PointCloud2Iterator<uint8_t> r_iter(cloud, "r");
  seerep_hdf5::PointCloud2Iterator<uint8_t> g_iter(cloud, "g");
  seerep_hdf5::PointCloud2Iterator<uint8_t> b_iter(cloud, "b");
  seerep_hdf5::PointCloud2Iterator<uint8_t> a_iter(cloud, "a");

  std::vector<std::vector<std::vector<uint8_t>>> color_data;
  std::string colors_id = HDF5_GROUP_POINTCLOUD + "/" + uuid + "/colors";

  HighFive::DataSet colors_dataset = m_file->getDataSet(colors_id);

  colors_dataset.read(color_data);

  for (auto column : color_data)
  {
    for (auto row : column)
    {
      *r_iter = row[0];
      *g_iter = row[1];
      *b_iter = row[2];
      *a_iter = row[3];

      ++r_iter;
      ++g_iter;
      ++b_iter;
      ++a_iter;
    }
  }
}

void PointCloudIO::writePointFieldAttributes(
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
PointCloudIO::readPointFieldAttributes(HighFive::Group& cloud_group)
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
