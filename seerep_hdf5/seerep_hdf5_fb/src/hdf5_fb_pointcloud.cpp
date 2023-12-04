#include "seerep_hdf5_fb/hdf5_fb_pointcloud.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_fb
{
Hdf5FbPointCloud::Hdf5FbPointCloud(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& writeMtx)
  : Hdf5CoreGeneral(file, writeMtx), Hdf5FbGeneral(file, writeMtx)
{
}

void Hdf5FbPointCloud::writePointCloud2(const std::string& id, const seerep::fb::PointCloud2& pcl)
{
  const std::scoped_lock lock(*m_write_mtx);

  const std::string hdf5GroupPath = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + id;
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
      << "writing flatbuffers point cloud to: " << hdf5GroupPath;

  std::shared_ptr<HighFive::Group> dataGroupPtr = getHdf5Group(hdf5GroupPath);

  writeGeneralAttributes(dataGroupPtr, pcl);

  writeHeaderAttributes(*dataGroupPtr, pcl.header());

  writePointFieldAttributes(*dataGroupPtr, pcl.fields());

  writeBoundingBoxLabeled(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD, id, pcl.labels_bb());
  writeLabelsGeneral(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD, id, pcl.labels_general());

  HighFive::DataSpace dataSpace{ pcl.data()->size() };
  std::shared_ptr<HighFive::DataSet> payloadPtr = getHdf5DataSet<uint8_t>(hdf5GroupPath + "/points", dataSpace);
  payloadPtr->write(std::move(pcl.data()->data()));
  m_file->flush();
}

void Hdf5FbPointCloud::writeBoundingBox(const std::string& uuid, const seerep_core_msgs::Point& min_corner,
                                        const seerep_core_msgs::Point& max_corner)
{
  std::shared_ptr<HighFive::Group> dataGroupPtr =
      getHdf5Group(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid);
  std::vector<float> bb{ min_corner.get<0>(), min_corner.get<1>(), min_corner.get<2>(),
                         max_corner.get<0>(), max_corner.get<1>(), max_corner.get<2>() };
  writeAttributeToHdf5<std::vector<float>>(*dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::BOUNDINGBOX, bb);
  m_file->flush();
}

std::pair<seerep_core_msgs::Point, seerep_core_msgs::Point>
Hdf5FbPointCloud::computeBoundingBox(const seerep::fb::PointCloud2& pcl)
{
  seerep_core_msgs::Point min_corner = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                                         std::numeric_limits<float>::max() };
  seerep_core_msgs::Point max_corner = { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                                         std::numeric_limits<float>::lowest() };

  // TODO: Do we really need to PCL iterator in this case?
  seerep_hdf5_fb::PointCloud2ConstIterator<float> x_ptr(pcl.data()->data(), getOffset(pcl, "x"), pcl.point_step(),
                                                        pcl.height(), pcl.width());
  seerep_hdf5_fb::PointCloud2ConstIterator<float> y_ptr(pcl.data()->data(), getOffset(pcl, "y"), pcl.point_step(),
                                                        pcl.height(), pcl.width());
  seerep_hdf5_fb::PointCloud2ConstIterator<float> z_ptr(pcl.data()->data(), getOffset(pcl, "z"), pcl.point_step(),
                                                        pcl.height(), pcl.width());

  for (size_t i = 0; i < pcl.data()->size(); i += pcl.point_step())
  {
    const float& x = *x_ptr;
    const float& y = *y_ptr;
    const float& z = *z_ptr;

    ++x_ptr, ++y_ptr, ++z_ptr;

    min_corner.set<0>(std::min(min_corner.get<0>(), x));
    min_corner.set<1>(std::min(min_corner.get<1>(), y));
    min_corner.set<2>(std::min(min_corner.get<2>(), z));

    max_corner.set<0>(std::max(max_corner.get<0>(), x));
    max_corner.set<1>(std::max(max_corner.get<1>(), y));
    max_corner.set<2>(std::max(max_corner.get<2>(), z));
  }
  return std::make_pair(min_corner, max_corner);
}

void Hdf5FbPointCloud::writeGeneralAttributes(std::shared_ptr<HighFive::Group>& dataGroupPtr,
                                              const seerep::fb::PointCloud2& cloud)
{
  writeAttributeToHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::HEIGHT, cloud.height());
  writeAttributeToHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::WIDTH, cloud.width());
  writeAttributeToHdf5<bool>(*dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::IS_BIGENDIAN, cloud.is_bigendian());
  writeAttributeToHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::POINT_STEP, cloud.point_step());
  writeAttributeToHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::ROW_STEP, cloud.row_step());
  writeAttributeToHdf5<bool>(*dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::IS_DENSE, cloud.is_dense());
}

void Hdf5FbPointCloud::writePointCloudBoundingBoxLabeled(
    const std::string& id,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeledWithCategory>>* bbLabeledWithCategory)
{
  const std::scoped_lock lock(*m_write_mtx);

  writeBoundingBoxLabeled(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD, id, bbLabeledWithCategory);
}

// TODO partial point cloud read
std::optional<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>
Hdf5FbPointCloud::readPointCloud2(const std::string& id, const bool withoutData)
{
  const std::scoped_lock lock(*m_write_mtx);

  const std::string hdf5GroupPath = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + id;
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
      << "loading flatbuffers point cloud from: " << hdf5GroupPath;

  std::shared_ptr<HighFive::Group> dataGroupPtr = getHdf5Group(hdf5GroupPath, false);

  if (!dataGroupPtr)
  {
    return std::nullopt;
  }

  flatbuffers::grpc::MessageBuilder builder;

  // read general attributes from data group
  uint32_t height, width, pointStep, rowStep;
  bool isBigendian, isDense;

  try
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
        << "loading general attributes of : " << hdf5GroupPath;
    readGeneralAttributes(id, dataGroupPtr, height, width, pointStep, rowStep, isBigendian, isDense);
  }
  catch (const std::invalid_argument& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "error loading general attributes of: " << hdf5GroupPath;
    return std::nullopt;
  }

  // reads and creates a flatbuffers header
  auto headerAttributesOffset = readHeaderAttributes(builder, *dataGroupPtr, id);

  // read point field attributes
  std::vector<std::string> names;
  std::vector<uint32_t> offsets, counts;
  std::vector<uint8_t> datatypes;

  try
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
        << "loading point field attributes of: " << hdf5GroupPath;
    readPointFields(id, dataGroupPtr, names, offsets, counts, datatypes);
  }
  catch (const std::invalid_argument& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "error loading point field attributes of: " << hdf5GroupPath;
    return std::nullopt;
  }

  auto pointFieldsVectorOffset = readPointFieldsOffset(builder, names, offsets, counts, datatypes);

  flatbuffers::uoffset_t dataOffset;
  uint8_t* data;
  if (!withoutData)
  {
    dataOffset = builder.CreateUninitializedVector(height * width * pointStep, sizeof(uint8_t), &data);
    HighFive::DataSet payloadDataset = m_file->getDataSet(hdf5GroupPath + "/points");
    payloadDataset.read(data);
  }

  auto labelsGeneralOffset =
      readGeneralLabels(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD, id, builder);

  auto boundingBoxLabeledOffset =
      readBoundingBoxesLabeled(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD, id, builder);

  // build the point cloud message
  seerep::fb::PointCloud2Builder pointCloudBuilder(builder);
  pointCloudBuilder.add_header(headerAttributesOffset);
  pointCloudBuilder.add_height(height);
  pointCloudBuilder.add_width(width);
  pointCloudBuilder.add_fields(pointFieldsVectorOffset);
  pointCloudBuilder.add_is_bigendian(isBigendian);
  pointCloudBuilder.add_point_step(pointStep);
  pointCloudBuilder.add_row_step(rowStep);
  if (!withoutData)
  {
    pointCloudBuilder.add_data(dataOffset);
  }
  pointCloudBuilder.add_is_dense(isDense);
  pointCloudBuilder.add_labels_general(labelsGeneralOffset);
  pointCloudBuilder.add_labels_bb(boundingBoxLabeledOffset);
  auto pointCloudOffset = pointCloudBuilder.Finish();
  builder.Finish(pointCloudOffset);

  auto grpcPointCloud = builder.ReleaseMessage<seerep::fb::PointCloud2>();
  return grpcPointCloud;
}

void Hdf5FbPointCloud::readGeneralAttributes(const std::string& id, std::shared_ptr<HighFive::Group> dataGroupPtr,
                                             uint32_t& height, uint32_t& width, uint32_t& pointStep, uint32_t& rowStep,
                                             bool& isBigendian, bool& isDense)
{
  height = readAttributeFromHdf5<uint32_t>(id, *dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::HEIGHT);
  width = readAttributeFromHdf5<uint32_t>(id, *dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::WIDTH);
  pointStep = readAttributeFromHdf5<uint32_t>(id, *dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::POINT_STEP);
  rowStep = readAttributeFromHdf5<uint32_t>(id, *dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::ROW_STEP);
  isBigendian = readAttributeFromHdf5<bool>(id, *dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::IS_BIGENDIAN);
  isDense = readAttributeFromHdf5<bool>(id, *dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::IS_DENSE);
}

void Hdf5FbPointCloud::readPointFields(const std::string& id, std::shared_ptr<HighFive::Group> dataGroupPtr,
                                       std::vector<std::string>& names, std::vector<uint32_t>& offsets,
                                       std::vector<uint32_t>& counts, std::vector<uint8_t>& datatypes)
{
  names = readAttributeFromHdf5<std::vector<std::string>>(id, *dataGroupPtr,
                                                          seerep_hdf5_core::Hdf5CorePointCloud::FIELD_NAME);
  offsets = readAttributeFromHdf5<std::vector<uint32_t>>(id, *dataGroupPtr,
                                                         seerep_hdf5_core::Hdf5CorePointCloud::FIELD_OFFSET);
  counts = readAttributeFromHdf5<std::vector<uint32_t>>(id, *dataGroupPtr,
                                                        seerep_hdf5_core::Hdf5CorePointCloud::FIELD_COUNT);
  datatypes = readAttributeFromHdf5<std::vector<uint8_t>>(id, *dataGroupPtr,
                                                          seerep_hdf5_core::Hdf5CorePointCloud::FIELD_DATATYPE);
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<seerep::fb::PointField>>>
Hdf5FbPointCloud::readPointFieldsOffset(flatbuffers::grpc::MessageBuilder& builder, std::vector<std::string>& names,
                                        std::vector<uint32_t>& offsets, std::vector<uint32_t>& counts,
                                        std::vector<uint8_t>& datatypes)
{
  std::vector<flatbuffers::Offset<seerep::fb::PointField>> pointFields;
  for (size_t i = 0; i < names.size(); i++)
  {
    auto nameStr = builder.CreateString(names.at(i));
    seerep::fb::PointFieldBuilder pointField(builder);
    pointField.add_name(nameStr);
    pointField.add_offset(offsets.at(i));
    pointField.add_datatype(static_cast<seerep::fb::Point_Field_Datatype>(datatypes.at(i)));
    pointField.add_count(counts.at(i));
    pointFields.push_back(pointField.Finish());
  }
  return builder.CreateVector(pointFields);
}

Hdf5FbPointCloud::CloudInfo Hdf5FbPointCloud::getCloudInfo(const seerep::fb::PointCloud2& cloud)
{
  bool hasFieldx = false;
  bool hasFieldy = false;
  bool hasFieldz = false;

  CloudInfo info;

  for (size_t i = 0; i < cloud.fields()->size(); i++)
  {
    const std::string& fieldName = cloud.fields()->Get(i)->name()->str();
    if (fieldName == "x")
    {
      hasFieldx = true;
    }
    else if (fieldName == "y")
    {
      hasFieldy = true;
    }
    else if (fieldName == "z")
    {
      hasFieldz = true;
    }
    else if (fieldName == "rgb")
    {
      info.has_rgb = true;
    }
    else if (fieldName == "rgba")
    {
      info.has_rgba = true;
    }
    else if (fieldName.find("normal") == 0)
    {
      info.has_normals = true;
    }
  }
  if (hasFieldx && hasFieldy && hasFieldz)
  {
    info.has_points = true;
  }
  return info;
}

Hdf5FbPointCloud::CloudInfo Hdf5FbPointCloud::getCloudInfo(const std::vector<std::string>& fields)
{
  bool hasFieldx = false;
  bool hasFieldy = false;
  bool hasFieldz = false;

  CloudInfo info;

  for (auto fieldName : fields)
  {
    if (fieldName == "x")
    {
      hasFieldx = true;
    }
    else if (fieldName == "y")
    {
      hasFieldy = true;
    }
    else if (fieldName == "z")
    {
      hasFieldz = true;
    }
    else if (fieldName == "rgb")
    {
      info.has_rgb = true;
    }
    else if (fieldName == "rgba")
    {
      info.has_rgba = true;
    }
    else if (fieldName.find("normal") == 0)
    {
      info.has_normals = true;
    }
  }
  if (hasFieldx && hasFieldy && hasFieldz)
  {
    info.has_points = true;
  }
  return info;
}

uint32_t Hdf5FbPointCloud::getOffset(const std::vector<std::string>& names, const std::vector<uint32_t>& offsets,
                                     const std::string& fieldName, bool isBigendian)
{
  for (size_t i = 0; i < names.size(); i++)
  {
    if (names[i] == fieldName)
    {
      return offsets[i];
    }
  }

  std::set<std::string> fieldNames = { "r", "g", "b", "a" };

  if (fieldNames.find(fieldName) != fieldNames.end())
  {
    for (size_t i = 0; i < names.size(); i++)
    {
      if (names[i] == "rgb" || names[i] == "rgba")
      {
        return rgbaOffset(fieldName, offsets[i], isBigendian);
      }
    }
  }
  throw std::runtime_error("Field " + fieldName + " does not exist!");
}

std::vector<uint32_t> Hdf5FbPointCloud::getOffsets(const std::vector<std::string>& names,
                                                   const std::vector<uint32_t>& offsets, bool isBigendian,
                                                   const std::vector<std::string>& fields)
{
  std::vector<uint32_t> calcOffsets;
  for (auto field : fields)
  {
    calcOffsets.push_back(getOffset(names, offsets, field, isBigendian));
  }
  return calcOffsets;
}

uint32_t Hdf5FbPointCloud::getOffset(const seerep::fb::PointCloud2& cloud, const std::string& fieldName)
{
  for (size_t i = 0; i < cloud.fields()->size(); i++)
  {
    const seerep::fb::PointField& field = *cloud.fields()->Get(i);
    if (field.name()->str() == fieldName)
    {
      return field.offset();
    }
  }

  std::set<std::string> field_names = { "r", "g", "b", "a" };

  if (field_names.find(fieldName) != field_names.end())
  {
    for (size_t i = 0; i < cloud.fields()->size(); i++)
    {
      const seerep::fb::PointField& field = *cloud.fields()->Get(i);

      if (field.name()->str() == "rgb" || field.name()->str() == "rgba")
      {
        return rgbaOffset(fieldName, field.offset(), cloud.is_bigendian());
      }
    }
  }
  throw std::runtime_error("Field " + fieldName + " does not exist!");
}

std::vector<uint32_t> Hdf5FbPointCloud::getOffsets(const seerep::fb::PointCloud2& cloud,
                                                   const std::vector<std::string>& fields)
{
  std::vector<uint32_t> calcOffsets;
  for (auto field : fields)
  {
    calcOffsets.push_back(getOffset(cloud, field));
  }
  return calcOffsets;
}

uint32_t Hdf5FbPointCloud::rgbaOffset(const std::string& fieldName, uint32_t offset, bool isBigendian)
{
  if (fieldName == "r")
  {
    if (isBigendian)
    {
      return offset + 1;
    }
    else
    {
      return offset + 2;
    }
  }
  if (fieldName == "g")
  {
    if (isBigendian)
    {
      return offset + 2;
    }
    else
    {
      return offset + 1;
    }
  }
  if (fieldName == "b")
  {
    if (isBigendian)
    {
      return offset + 3;
    }
    else
    {
      return offset + 0;
    }
  }
  if (fieldName == "a")
  {
    if (isBigendian)
    {
      return offset + 0;
    }
    else
    {
      return offset + 3;
    }
  }
  throw std::runtime_error("Field " + fieldName + " does not exist!");
}

}  // namespace seerep_hdf5_fb
