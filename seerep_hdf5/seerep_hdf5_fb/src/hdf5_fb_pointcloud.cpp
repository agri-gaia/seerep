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

  writeLabelsFb(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD, id, pcl.labels());

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

  const float* x = reinterpret_cast<const float*>(pcl.data()->data() + getChannelOffset(pcl, "x"));
  const float* y = reinterpret_cast<const float*>(pcl.data()->data() + getChannelOffset(pcl, "y"));
  const float* z = reinterpret_cast<const float*>(pcl.data()->data() + getChannelOffset(pcl, "z"));

  size_t inc = pcl.point_step() / sizeof(float);
  for (size_t i = 0; i < pcl.data()->size(); i += pcl.point_step())
  {
    min_corner.set<0>(std::min(min_corner.get<0>(), *x));
    min_corner.set<1>(std::min(min_corner.get<1>(), *y));
    min_corner.set<2>(std::min(min_corner.get<2>(), *z));

    max_corner.set<0>(std::max(max_corner.get<0>(), *x));
    max_corner.set<1>(std::max(max_corner.get<1>(), *y));
    max_corner.set<2>(std::max(max_corner.get<2>(), *z));

    x += inc, y += inc, z += inc;
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

  auto labelsOffset = readLabels(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD, id, builder);

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
  pointCloudBuilder.add_labels(labelsOffset);
  auto pointCloudOffset = pointCloudBuilder.Finish();
  builder.Finish(pointCloudOffset);

  auto grpcPointCloud = builder.ReleaseMessage<seerep::fb::PointCloud2>();
  return grpcPointCloud;
}

void Hdf5FbPointCloud::readGeneralAttributes(const std::string& id, std::shared_ptr<HighFive::Group> dataGroupPtr,
                                             uint32_t& height, uint32_t& width, uint32_t& pointStep, uint32_t& rowStep,
                                             bool& isBigendian, bool& isDense)
{
  height = readAttributeFromHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::HEIGHT, id);
  width = readAttributeFromHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::WIDTH, id);
  pointStep = readAttributeFromHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::POINT_STEP, id);
  rowStep = readAttributeFromHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::ROW_STEP, id);
  isBigendian = readAttributeFromHdf5<bool>(*dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::IS_BIGENDIAN, id);
  isDense = readAttributeFromHdf5<bool>(*dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::IS_DENSE, id);
}

void Hdf5FbPointCloud::readPointFields(const std::string& id, std::shared_ptr<HighFive::Group> dataGroupPtr,
                                       std::vector<std::string>& names, std::vector<uint32_t>& offsets,
                                       std::vector<uint32_t>& counts, std::vector<uint8_t>& datatypes)
{
  names = readAttributeFromHdf5<std::vector<std::string>>(*dataGroupPtr,
                                                          seerep_hdf5_core::Hdf5CorePointCloud::FIELD_NAME, id);
  offsets = readAttributeFromHdf5<std::vector<uint32_t>>(*dataGroupPtr,
                                                         seerep_hdf5_core::Hdf5CorePointCloud::FIELD_OFFSET, id);
  counts = readAttributeFromHdf5<std::vector<uint32_t>>(*dataGroupPtr,
                                                        seerep_hdf5_core::Hdf5CorePointCloud::FIELD_COUNT, id);
  datatypes = readAttributeFromHdf5<std::vector<uint8_t>>(*dataGroupPtr,
                                                          seerep_hdf5_core::Hdf5CorePointCloud::FIELD_DATATYPE, id);
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

uint32_t Hdf5FbPointCloud::getChannelOffset(const seerep::fb::PointCloud2& pcl, const std::string& channel_name) const
{
  for (size_t i = 0; i < pcl.fields()->size(); i++)
  {
    const seerep::fb::PointField* field = pcl.fields()->Get(i);
    if (field->name()->str() == channel_name)
    {
      return field->offset();
    }
    else if (field->name()->str() == "rgb" || field->name()->str() == "rgba")
    {
      return getRgbaOffset(channel_name, field->offset(), pcl.is_bigendian());
    }
  }
  throw std::runtime_error("Requested field: " + channel_name + " does not exist!");
}

uint32_t Hdf5FbPointCloud::getRgbaOffset(const std::string& channel_name, uint32_t base_offset, bool is_big_endian) const
{
  if (channel_name == "r")
  {
    return is_big_endian ? base_offset + 1 : base_offset + 2;
  }
  if (channel_name == "g")
  {
    return is_big_endian ? base_offset + 2 : base_offset + 1;
  }
  if (channel_name == "b")
  {
    return is_big_endian ? base_offset + 3 : base_offset + 0;
  }
  if (channel_name == "a")
  {
    return is_big_endian ? base_offset + 0 : base_offset + 3;
  }
  throw std::runtime_error("Requested field: " + channel_name + " does not exist!");
}

}  // namespace seerep_hdf5_fb
