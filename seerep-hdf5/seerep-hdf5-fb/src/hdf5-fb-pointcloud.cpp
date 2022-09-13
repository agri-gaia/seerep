#include "seerep-hdf5-fb/hdf5-fb-pointcloud.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_fb
{
Hdf5FbPointCloud::Hdf5FbPointCloud(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5FbGeneral(file, write_mtx)
{
}

std::map<std::string, HighFive::Group> Hdf5FbPointCloud::getPointClouds()
{
  const std::scoped_lock lock(*m_write_mtx);

  std::map<std::string, HighFive::Group> map;
  if (m_file->exist(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD))
  {
    const HighFive::Group& clouds_group = m_file->getGroup(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD);
    for (std::string& cloud_name : clouds_group.listObjectNames())
    {
      map.insert(std::pair(cloud_name, clouds_group.getGroup(cloud_name)));
    }
  }
  return map;
}

std::shared_ptr<HighFive::Group> Hdf5FbPointCloud::writePointCloud2(const std::string& uuid,
                                                                    const seerep::fb::PointCloud2* pointcloud2)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string cloud_group_id = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid;

  std::shared_ptr<HighFive::Group> data_group_ptr;

  if (!m_file->exist(cloud_group_id))
  {
    data_group_ptr = std::make_shared<HighFive::Group>(m_file->createGroup(cloud_group_id));
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::HEIGHT, pointcloud2->height());
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::WIDTH, pointcloud2->width());
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_BIGENDIAN, pointcloud2->is_bigendian());
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::POINT_STEP, pointcloud2->point_step());
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::ROW_STEP, pointcloud2->row_step());
    data_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_DENSE, pointcloud2->is_dense());
  }
  else
  {
    data_group_ptr = std::make_shared<HighFive::Group>(m_file->getGroup(cloud_group_id));
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::HEIGHT).write(pointcloud2->height());
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::WIDTH).write(pointcloud2->width());
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_BIGENDIAN).write(pointcloud2->is_bigendian());
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::POINT_STEP).write(pointcloud2->point_step());
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::ROW_STEP).write(pointcloud2->row_step());
    data_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_DENSE).write(pointcloud2->is_dense());
  }

  writePointFieldAttributes(*data_group_ptr, *pointcloud2->fields());
  writeHeaderAttributes(*data_group_ptr, *pointcloud2->header());

  if (flatbuffers::IsFieldPresent(pointcloud2, seerep::fb::PointCloud2::VT_LABELS_GENERAL))
    writeLabelsGeneral(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD, uuid,
                       *pointcloud2->labels_general());
  if (flatbuffers::IsFieldPresent(pointcloud2, seerep::fb::PointCloud2::VT_LABELS_BB))
    writeBoundingBoxLabeled(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD, uuid,
                            *pointcloud2->labels_bb());

  CloudInfo info = getCloudInfo(*pointcloud2);

  if (info.has_points)
    writePoints(uuid, data_group_ptr, *pointcloud2);
  if (info.has_rgb)
    writeColorsRGB(uuid, *pointcloud2);
  if (info.has_rgba)
    writeColorsRGBA(uuid, *pointcloud2);

  // TODO normals
  if (!info.other_fields.empty())
    writeOtherFields(uuid, *pointcloud2, info.other_fields);

  m_file->flush();
  return data_group_ptr;
}

void Hdf5FbPointCloud::writePoints(const std::string& uuid, const std::shared_ptr<HighFive::Group>& data_group_ptr,
                                   const seerep::fb::PointCloud2& cloud)
{
  std::string points_id = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid + "/points";
  HighFive::DataSpace data_space({ cloud.height(), cloud.width(), 3 });

  std::shared_ptr<HighFive::DataSet> points_dataset_ptr;
  if (!m_file->exist(points_id))
    points_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->createDataSet<float>(points_id, data_space));
  else
    points_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(points_id));

  std::vector<std::vector<std::vector<float>>> point_data;
  point_data.resize(cloud.height());

  seerep_hdf5_fb::PointCloud2ConstIterator<float> x_iter(cloud, "x");
  seerep_hdf5_fb::PointCloud2ConstIterator<float> y_iter(cloud, "y");
  seerep_hdf5_fb::PointCloud2ConstIterator<float> z_iter(cloud, "z");

  std::array<float, 3> min, max;
  min[0] = min[1] = min[2] = std::numeric_limits<float>::max();
  max[0] = max[1] = max[2] = std::numeric_limits<float>::min();

  for (unsigned int i = 0; i < cloud.height(); i++)
  {
    point_data[i].reserve(cloud.width());
    for (unsigned int j = 0; j < cloud.width(); j++)
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

  writeAttributeToHdf5(*data_group_ptr, seerep_hdf5_core::Hdf5CorePointCloud::BOUNDINGBOX, boundingbox);

  // write data to dataset
  points_dataset_ptr->write(point_data);
}

void Hdf5FbPointCloud::writeColorsRGB(const std::string& uuid, const seerep::fb::PointCloud2& cloud)
{
  const std::string colors_id = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid + "/colors";
  HighFive::DataSpace data_space({ cloud.height(), cloud.width(), 3 });

  std::shared_ptr<HighFive::DataSet> colors_dataset_ptr;
  if (!m_file->exist(colors_id))
    colors_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->createDataSet<uint8_t>(colors_id, data_space));
  else
    colors_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(colors_id));

  std::vector<std::vector<std::vector<uint8_t>>> colors_data;
  colors_data.resize(cloud.height());

  seerep_hdf5_fb::PointCloud2ConstIterator<uint8_t> r_iter(cloud, "r");
  seerep_hdf5_fb::PointCloud2ConstIterator<uint8_t> g_iter(cloud, "g");
  seerep_hdf5_fb::PointCloud2ConstIterator<uint8_t> b_iter(cloud, "b");

  for (unsigned int i = 0; i < cloud.height(); i++)
  {
    colors_data[i].reserve(cloud.width());
    for (unsigned int j = 0; j < cloud.width(); j++)
    {
      colors_data[i].push_back(std::vector{ *r_iter, *g_iter, *b_iter });
      ++r_iter, ++g_iter, ++b_iter;
    }
  }

  colors_dataset_ptr->write(colors_data);
}

void Hdf5FbPointCloud::writeColorsRGBA(const std::string& uuid, const seerep::fb::PointCloud2& cloud)
{
  const std::string colors_id = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid + "/colors";
  HighFive::DataSpace data_space({ cloud.height(), cloud.width(), 4 });

  std::shared_ptr<HighFive::DataSet> colors_dataset_ptr;
  if (!m_file->exist(colors_id))
    colors_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->createDataSet<uint8_t>(colors_id, data_space));
  else
    colors_dataset_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(colors_id));

  std::vector<std::vector<std::vector<uint8_t>>> colors_data;
  colors_data.resize(cloud.height());

  seerep_hdf5_fb::PointCloud2ConstIterator<uint8_t> r_iter(cloud, "r");
  seerep_hdf5_fb::PointCloud2ConstIterator<uint8_t> g_iter(cloud, "g");
  seerep_hdf5_fb::PointCloud2ConstIterator<uint8_t> b_iter(cloud, "b");
  seerep_hdf5_fb::PointCloud2ConstIterator<uint8_t> a_iter(cloud, "a");

  for (unsigned int i = 0; i < cloud.height(); i++)
  {
    colors_data[i].reserve(cloud.width());
    for (unsigned int j = 0; j < cloud.width(); j++)
    {
      colors_data[i].push_back(std::vector{ *r_iter, *g_iter, *b_iter, *a_iter });
      ++r_iter;
      ++g_iter;
      ++b_iter;
      ++a_iter;
    }
  }

  colors_dataset_ptr->write(colors_data);
}

void Hdf5FbPointCloud::writeOtherFields(const std::string& uuid, const seerep::fb::PointCloud2& cloud,
                                        const std::map<std::string, PointFieldInfo>& fields)
{
  for (auto map_entry : fields)
  {
    const std::string name = map_entry.first;
    const PointFieldInfo info = map_entry.second;
    switch (info.datatype)
    {
      case seerep::fb::Point_Field_Datatype_INT8:
        write<int8_t>(uuid, name, cloud, info.count);
        break;
      case seerep::fb::Point_Field_Datatype_UINT8:
        write<uint8_t>(uuid, name, cloud, info.count);
        break;
      case seerep::fb::Point_Field_Datatype_INT16:
        write<int16_t>(uuid, name, cloud, info.count);
        break;
      case seerep::fb::Point_Field_Datatype_UINT16:
        write<uint16_t>(uuid, name, cloud, info.count);
        break;
      case seerep::fb::Point_Field_Datatype_INT32:
        write<int32_t>(uuid, name, cloud, info.count);
        break;
      case seerep::fb::Point_Field_Datatype_UINT32:
        write<uint32_t>(uuid, name, cloud, info.count);
        break;
      case seerep::fb::Point_Field_Datatype_FLOAT32:
        write<float>(uuid, name, cloud, info.count);
        break;
      case seerep::fb::Point_Field_Datatype_FLOAT64:
        write<double>(uuid, name, cloud, info.count);
        break;
      default:
        std::cout << "datatype of pointcloud unknown" << std::endl;
        break;
    }
  }
}

void Hdf5FbPointCloud::writePointFieldAttributes(
    HighFive::Group& cloud_group,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::PointField>>& vectorPointField)
{
  std::vector<std::string> names;
  std::vector<uint32_t> offsets, counts;
  std::vector<uint8_t> datatypes;
  for (size_t i = 0; i < vectorPointField.size(); i++)
  {
    names.push_back(vectorPointField.Get(i)->name()->str());
    offsets.push_back(vectorPointField.Get(i)->offset());
    datatypes.push_back(static_cast<uint8_t>(vectorPointField.Get(i)->datatype()));
    counts.push_back(vectorPointField.Get(i)->count());
  }
  writeAttributeToHdf5<std::vector<std::string>>(cloud_group, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_NAME, names);

  writeAttributeToHdf5<std::vector<uint32_t>>(cloud_group, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_OFFSET, offsets);

  writeAttributeToHdf5<std::vector<uint8_t>>(cloud_group, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_DATATYPE,
                                             datatypes);

  writeAttributeToHdf5<std::vector<uint32_t>>(cloud_group, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_COUNT, counts);
}

Hdf5FbPointCloud::CloudInfo Hdf5FbPointCloud::getCloudInfo(const seerep::fb::PointCloud2& cloud)
{
  bool hasFieldx = false;
  bool hasFieldy = false;
  bool hasFieldz = false;

  CloudInfo info;
  for (size_t i = 0; i < cloud.fields()->size(); i++)
  {
    const std::string fieldName = cloud.fields()->Get(i)->name()->str();
    if (fieldName == "x")
      hasFieldx = true;
    else if (fieldName == "y")
      hasFieldy = true;
    else if (fieldName == "z")
      hasFieldz = true;
    else if (fieldName == "rgb")
      info.has_rgb = true;
    else if (fieldName == "rgba")
      info.has_rgba = true;
    else if (fieldName.find("normal") == 0)
      info.has_normals = true;
    else
    {
      info.other_fields[fieldName] = (struct PointFieldInfo){ .datatype = cloud.fields()->Get(i)->datatype(),
                                                              .count = cloud.fields()->Get(i)->count() };
    }
  }
  if (hasFieldx && hasFieldy && hasFieldz)
    info.has_points = true;
  return info;
}

// TODO read partial point cloud, e.g. only xyz without color, etc.
std::optional<seerep::fb::PointCloud2> Hdf5FbPointCloud::readPointCloud2(const std::string& uuid)
{
  // const std::scoped_lock lock(*m_write_mtx);

  // if (!m_file->exist(uuid))
  // {
  //   return std::nullopt;
  // }
  // HighFive::Group cloud_group = m_file->getGroup(uuid);

  // seerep::fb::PointCloud2 pointcloud2;

  // uint32_t height, width, point_step, row_step;
  // bool is_bigendian, is_dense;
  // cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::HEIGHT).read(height);
  // cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::WIDTH).read(width);
  // cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_BIGENDIAN).read(is_bigendian);
  // cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::POINT_STEP).read(point_step);
  // cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::ROW_STEP).read(row_step);
  // cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::IS_DENSE).read(is_dense);

  // auto pointFieldAttributes = readPointFieldAttributes(cloud_group);
  // auto headerAttributes = readHeaderAttributes(cloud_group);

  // // TODO build header and Point Fields

  // CloudInfo info = getCloudInfo(pointcloud2);

  //   if (info.has_points)
  //     readPoints(uuid, pointcloud2);

  //   if (info.has_rgb)
  //     readColorsRGB(uuid, pointcloud2);

  //   if (info.has_rgba)
  //     readColorsRGBA(uuid, pointcloud2);

  //   // TODO normals

  //   if (!info.other_fields.empty())
  //     readOtherFields(uuid, pointcloud2, info.other_fields);

  //   return pointcloud2;
}

void Hdf5FbPointCloud::readPoints(const std::string& uuid, seerep::fb::PointCloud2& cloud)
{
  //   seerep_hdf5_pb::PointCloud2Iterator<float> x_iter(cloud, "x");
  //   seerep_hdf5_pb::PointCloud2Iterator<float> y_iter(cloud, "y");
  //   seerep_hdf5_pb::PointCloud2Iterator<float> z_iter(cloud, "z");

  //   std::vector<std::vector<std::vector<float>>> point_data;
  //   std::string points_id = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid + "/points";

  //   HighFive::DataSet points_dataset = m_file->getDataSet(points_id);

  //   points_dataset.read(point_data);

  //   for (auto column : point_data)
  //   {
  //     for (auto row : column)
  //     {
  //       *x_iter = row[0];
  //       *y_iter = row[1];
  //       *z_iter = row[2];
  //       ++x_iter, ++y_iter, ++z_iter;
  //     }
  //   }
}

void Hdf5FbPointCloud::readColorsRGB(const std::string& uuid, seerep::fb::PointCloud2& cloud)
{
  // seerep_hdf5_pb::PointCloud2Iterator<uint8_t> r_iter(cloud, "r");
  // seerep_hdf5_pb::PointCloud2Iterator<uint8_t> g_iter(cloud, "g");
  // seerep_hdf5_pb::PointCloud2Iterator<uint8_t> b_iter(cloud, "b");

  // std::vector<std::vector<std::vector<uint8_t>>> color_data;
  // std::string colors_id = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid + "/colors";

  // HighFive::DataSet colors_dataset = m_file->getDataSet(colors_id);

  // colors_dataset.read(color_data);

  // for (auto column : color_data)
  // {
  //   for (auto row : column)
  //   {
  //     *r_iter = row[0];
  //     *g_iter = row[1];
  //     *b_iter = row[2];
  //     ++r_iter, ++g_iter, ++b_iter;
  //   }
  // }
}

void Hdf5FbPointCloud::readColorsRGBA(const std::string& uuid, seerep::fb::PointCloud2& cloud)
{
  // seerep_hdf5_pb::PointCloud2Iterator<uint8_t> r_iter(cloud, "r");
  // seerep_hdf5_pb::PointCloud2Iterator<uint8_t> g_iter(cloud, "g");
  // seerep_hdf5_pb::PointCloud2Iterator<uint8_t> b_iter(cloud, "b");
  // seerep_hdf5_pb::PointCloud2Iterator<uint8_t> a_iter(cloud, "a");

  // std::vector<std::vector<std::vector<uint8_t>>> color_data;
  // std::string colors_id = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid + "/colors";

  // HighFive::DataSet colors_dataset = m_file->getDataSet(colors_id);

  // colors_dataset.read(color_data);

  // for (auto column : color_data)
  // {
  //   for (auto row : column)
  //   {
  //     *r_iter = row[0];
  //     *g_iter = row[1];
  //     *b_iter = row[2];
  //     *a_iter = row[3];

  //     ++r_iter;
  //     ++g_iter;
  //     ++b_iter;
  //     ++a_iter;
  //   }
  // }
}

void Hdf5FbPointCloud::readOtherFields(const std::string& uuid, seerep::fb::PointCloud2& cloud,
                                       const std::map<std::string, PointFieldInfo>& fields)
{
  // for (auto field_map_entry : fields)
  // {
  //   const auto& field = field_map_entry.second;
  //   switch (field.datatype())
  //   {
  //     case seerep::PointField::INT8:
  //       read<int8_t>(uuid, field.name(), cloud, field.count());
  //       break;
  //     case seerep::PointField::UINT8:
  //       read<uint8_t>(uuid, field.name(), cloud, field.count());
  //       break;
  //     case seerep::PointField::INT16:
  //       read<int16_t>(uuid, field.name(), cloud, field.count());
  //       break;
  //     case seerep::PointField::UINT16:
  //       read<uint16_t>(uuid, field.name(), cloud, field.count());
  //       break;
  //     case seerep::PointField::INT32:
  //       read<int32_t>(uuid, field.name(), cloud, field.count());
  //       break;
  //     case seerep::PointField::UINT32:
  //       read<uint32_t>(uuid, field.name(), cloud, field.count());
  //       break;
  //     case seerep::PointField::FLOAT32:
  //       read<float>(uuid, field.name(), cloud, field.count());
  //       break;
  //     case seerep::PointField::FLOAT64:
  //       read<double>(uuid, field.name(), cloud, field.count());
  //       break;
  //     default:
  //       std::cout << "datatype of pointcloud unknown" << std::endl;
  //       break;
  //   }
  // }
}

flatbuffers::Vector<flatbuffers::Offset<seerep::fb::PointField>>
Hdf5FbPointCloud::readPointFieldAttributes(HighFive::Group& cloud_group)
{
  // google::protobuf::RepeatedPtrField<seerep::PointField> repeatedPointField;

  // std::vector<std::string> names;
  // std::vector<uint32_t> offsets, counts;
  // std::vector<uint8_t> datatypes;

  // cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::FIELD_NAME).read(names);
  // cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::FIELD_OFFSET).read(offsets);
  // cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::BOUNDINGBOX).read(datatypes);
  // cloud_group.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::BOUNDINGBOX).read(counts);

  // for (long unsigned int i = 0; i < names.size(); i++)
  // {
  //   seerep::PointField point_field;

  //   point_field.set_name(names.at(i));
  //   point_field.set_offset(offsets.at(i));
  //   point_field.set_datatype(static_cast<seerep::PointField::Datatype>(datatypes.at(i)));
  //   point_field.set_count(counts.at(i));

  //   *repeatedPointField.Add() = point_field;
  // }

  // return repeatedPointField;
}

std::vector<float> Hdf5FbPointCloud::loadBoundingBox(const std::string& uuid)
{
  // const std::scoped_lock lock(*m_write_mtx);

  // std::string hdf5DatasetPath = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid;
  // std::shared_ptr<HighFive::Group> group_ptr =
  // std::make_shared<HighFive::Group>(m_file->getGroup(hdf5DatasetPath)); std::vector<float> bb;
  // group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::BOUNDINGBOX).write(bb);
  // return bb;
}
}  // namespace seerep_hdf5_fb
