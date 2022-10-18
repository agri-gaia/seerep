#include "seerep-hdf5-fb/hdf5-fb-pointcloud.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_fb
{
Hdf5FbPointCloud::Hdf5FbPointCloud(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& writeMtx)
  : Hdf5FbGeneral(file, writeMtx)
{
}

void Hdf5FbPointCloud::writePointCloud2(const std::string& id, const seerep::fb::PointCloud2& cloud,
                                        std::vector<float>& boundingBox)
{
  const std::scoped_lock lock(*m_write_mtx);

  const std::string hdf5GroupPath = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + id;
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
      << "writing flatbuffers point cloud to: " << hdf5GroupPath;

  std::shared_ptr<HighFive::Group> dataGroupPtr = getHdf5Group(hdf5GroupPath);

  writeGeneralAttributes(dataGroupPtr, cloud);

  writeHeaderAttributes(*dataGroupPtr, cloud.header());

  writePointFieldAttributes(*dataGroupPtr, cloud.fields());

  writeBoundingBoxLabeled(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD, id, cloud.labels_bb());
  writeLabelsGeneral(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD, id, cloud.labels_general());

  CloudInfo info = getCloudInfo(cloud);

  if (info.has_points)
  {
    writePoints(id, getOffsets(cloud, std::vector<std::string>{ "x", "y", "z" }), cloud.data()->data(),
                cloud.point_step(), cloud.height(), cloud.width(), dataGroupPtr, boundingBox);
  }

  if (info.has_rgb)
  {
    writeColorsRGBA(id, getOffsets(cloud, std::vector<std::string>{ "r", "g", "b" }), cloud.data()->data(),
                    cloud.point_step(), cloud.height(), cloud.width());
  }

  if (info.has_rgba)
  {
    writeColorsRGBA(id, getOffsets(cloud, std::vector<std::string>{ "r", "g", "b", "a" }), cloud.data()->data(),
                    cloud.point_step(), cloud.height(), cloud.width());
  }

  // TODO normals

  // TODO other fields

  m_file->flush();
}

void Hdf5FbPointCloud::computeBoundingBox(std::array<float, 3>& min, std::array<float, 3>& max, const float& x,
                                          const float& y, const float& z)
{
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
}

void Hdf5FbPointCloud::writePoints(const std::string& id, const std::vector<uint32_t>& offsets, const uint8_t* data,
                                   uint32_t pointStep, uint32_t height, uint32_t width,
                                   const std::shared_ptr<HighFive::Group>& groupPtr, std::vector<float>& boundingBox)
{
  const std::string hdf5PointsPath = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + id + "/points";
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "writing dataset to: " << hdf5PointsPath;

  HighFive::DataSpace dataSpace({ height, width, 3 });
  std::shared_ptr<HighFive::DataSet> pointsDatasetPtr = getHdf5DataSet<float>(hdf5PointsPath, dataSpace);

  std::vector<std::vector<std::vector<float>>> pointData;
  pointData.resize(height);

  std::array<float, 3> min, max;
  min[0] = min[1] = min[2] = std::numeric_limits<float>::max();
  max[0] = max[1] = max[2] = std::numeric_limits<float>::min();

  seerep_hdf5_fb::PointCloud2ConstIterator<float> xIter(data, offsets[0], pointStep, height, width);
  seerep_hdf5_fb::PointCloud2ConstIterator<float> yIter(data, offsets[1], pointStep, height, width);
  seerep_hdf5_fb::PointCloud2ConstIterator<float> zIter(data, offsets[2], pointStep, height, width);

  for (size_t row = 0; row < height; row++)
  {
    pointData[row].reserve(width);
    for (size_t col = 0; col < width; col++)
    {
      const float& x = *xIter;
      const float& y = *yIter;
      const float& z = *zIter;

      computeBoundingBox(min, max, x, y, z);

      pointData[row].push_back(std::vector{ x, y, z });

      ++xIter, ++yIter, ++zIter;
    }
  }

  // min (x, y, z), max (x,y,z)
  boundingBox.insert(boundingBox.end(), { min[0], min[1], min[2], max[0], max[1], max[2] });

  // write bounding box as data group attribute
  writeAttributeToHdf5(*groupPtr, seerep_hdf5_core::Hdf5CorePointCloud::BOUNDINGBOX, boundingBox);

  pointsDatasetPtr->write(pointData);
}

void Hdf5FbPointCloud::writeColorsRGB(const std::string& id, const std::vector<uint32_t>& offsets, const uint8_t* data,
                                      uint32_t pointStep, uint32_t height, uint32_t width)
{
  const std::string hdf5ColorsPath = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + id + "/colors";
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "writing dataset to: " << hdf5ColorsPath;

  HighFive::DataSpace dataSpace({ height, width, 3 });
  std::shared_ptr<HighFive::DataSet> colorsDatasetPtr = getHdf5DataSet<float>(hdf5ColorsPath, dataSpace);

  std::vector<std::vector<std::vector<uint8_t>>> colorsData;
  colorsData.resize(height);

  seerep_hdf5_fb::PointCloud2ConstIterator<uint8_t> rIter(data, offsets[0], pointStep, height, width);
  seerep_hdf5_fb::PointCloud2ConstIterator<uint8_t> gIter(data, offsets[1], pointStep, height, width);
  seerep_hdf5_fb::PointCloud2ConstIterator<uint8_t> bIter(data, offsets[2], pointStep, height, width);

  for (size_t row = 0; row < height; row++)
  {
    colorsData[row].reserve(width);
    for (size_t col = 0; col < width; col++)
    {
      colorsData[row].push_back(std::vector{ *rIter, *gIter, *bIter });
      ++rIter, ++gIter, ++bIter;
    }
  }

  colorsDatasetPtr->write(colorsData);
}

void Hdf5FbPointCloud::writeColorsRGBA(const std::string& id, const std::vector<uint32_t>& offsets, const uint8_t* data,
                                       uint32_t pointStep, uint32_t height, uint32_t width)
{
  const std::string hdf5ColorsPath = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + id + "/colors";
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "writing dataset to: " << hdf5ColorsPath;

  HighFive::DataSpace dataSpace({ height, width, 4 });

  std::shared_ptr<HighFive::DataSet> colorsDatasetPtr = getHdf5DataSet<float>(hdf5ColorsPath, dataSpace);

  std::vector<std::vector<std::vector<uint8_t>>> colorsData;
  colorsData.resize(height);

  seerep_hdf5_fb::PointCloud2ConstIterator<uint8_t> rIter(data, offsets[0], pointStep, height, width);
  seerep_hdf5_fb::PointCloud2ConstIterator<uint8_t> gIter(data, offsets[1], pointStep, height, width);
  seerep_hdf5_fb::PointCloud2ConstIterator<uint8_t> bIter(data, offsets[2], pointStep, height, width);
  seerep_hdf5_fb::PointCloud2ConstIterator<uint8_t> aIter(data, offsets[3], pointStep, height, width);

  for (size_t row = 0; row < height; row++)
  {
    colorsData[row].reserve(width);
    for (size_t col = 0; col < width; col++)
    {
      colorsData[row].push_back(std::vector{ *rIter, *gIter, *bIter, *aIter });
      ++rIter;
      ++gIter;
      ++bIter;
      ++aIter;
    }
  }

  colorsDatasetPtr->write(colorsData);
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

// TODO partial point cloud read
std::optional<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>
Hdf5FbPointCloud::readPointCloud2(const std::string& id, const bool withoutData)
{
  const std::scoped_lock lock(*m_write_mtx);

  const std::string hdf5GroupPath = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + id;
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
      << "loading flatbuffers point cloud from: " << hdf5GroupPath;

  std::shared_ptr<HighFive::Group> dataGroupPtr;

  try
  {
    checkExists(hdf5GroupPath);
    dataGroupPtr = std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));
  }
  catch (std::invalid_argument const& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "hdf5 group " << hdf5GroupPath << " does not exist!";
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
  if (!withoutData)
  {
    // get info about the point cloud
    CloudInfo info = getCloudInfo(names);

    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
        << "reading point cloud data of: " << hdf5GroupPath;

    // pointer to the pre-allocated array
    uint8_t* data;
    // allocate height * width * pointSetp bytes
    dataOffset = builder.CreateUninitializedVector(height * width * pointStep, sizeof(uint8_t), &data);

    if (info.has_points)
    {
      readPoints(id, getOffsets(names, offsets, isBigendian, std::vector<std::string>{ "x", "y", "z" }), data,
                 pointStep, height, width);
    }

    if (info.has_rgb)
    {
      readColorsRGB(id, getOffsets(names, offsets, isBigendian, std::vector<std::string>{ "r", "g", "b" }), data,
                    pointStep, height, width);
    }

    if (info.has_rgba)
    {
      readPoints(id, getOffsets(names, offsets, isBigendian, std::vector<std::string>{ "r", "g", "b", "a" }), data,
                 pointStep, height, width);
    }
  }

  // TODO add normals

  // TODO add other fields

  auto generalLabelsOffset = readLabelsGeneralOffset(builder, id);

  auto boundingBoxLabeledOffset = readLabelsBoundingBoxOffset(builder, id);

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
  pointCloudBuilder.add_labels_general(generalLabelsOffset);
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

void Hdf5FbPointCloud::readPoints(const std::string& id, const std::vector<uint32_t>& offsets, uint8_t* data,
                                  uint32_t pointStep, uint32_t height, uint32_t width)
{
  const std::string hdf5DatasetPointsPath =
      seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + id + "/points";

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
      << "reading points from: " << hdf5DatasetPointsPath;

  HighFive::DataSet pointsDataset = m_file->getDataSet(hdf5DatasetPointsPath);

  std::vector<std::vector<std::vector<float>>> pointData;
  pointsDataset.read(pointData);

  seerep_hdf5_fb::PointCloud2Iterator<float> xIter(data, offsets[0], pointStep, height, width);
  seerep_hdf5_fb::PointCloud2Iterator<float> yIter(data, offsets[1], pointStep, height, width);
  seerep_hdf5_fb::PointCloud2Iterator<float> zIter(data, offsets[2], pointStep, height, width);

  for (auto col : pointData)
  {
    for (auto row : col)
    {
      *xIter = row[0];
      *yIter = row[1];
      *zIter = row[2];
      ++xIter, ++yIter, ++zIter;
    }
  }
}

void Hdf5FbPointCloud::readColorsRGB(const std::string& id, const std::vector<uint32_t>& offsets, uint8_t* data,
                                     uint32_t pointStep, uint32_t height, uint32_t width)
{
  const std::string hdf5DatasetPointsPath =
      seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + id + "/colors";

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "reading rgb from: " << hdf5DatasetPointsPath;

  HighFive::DataSet colorsDataset = m_file->getDataSet(hdf5DatasetPointsPath);

  std::vector<std::vector<std::vector<float>>> colorData;
  colorsDataset.read(colorData);

  seerep_hdf5_fb::PointCloud2Iterator<uint8_t> rIter(data, offsets[0], pointStep, height, width);
  seerep_hdf5_fb::PointCloud2Iterator<uint8_t> gIter(data, offsets[1], pointStep, height, width);
  seerep_hdf5_fb::PointCloud2Iterator<uint8_t> bIter(data, offsets[2], pointStep, height, width);

  for (auto col : colorData)
  {
    for (auto row : col)
    {
      *rIter = row[0];
      *gIter = row[1];
      *bIter = row[2];
      ++rIter, ++gIter, ++bIter;
    }
  }
}

void Hdf5FbPointCloud::readColorsRGBA(const std::string& id, const std::vector<uint32_t>& offsets, uint8_t* data,
                                      uint32_t pointStep, uint32_t height, uint32_t width)
{
  const std::string hdf5DatasetPointsPath =
      seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + id + "/colors";

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "reading rgba from: " << hdf5DatasetPointsPath;

  HighFive::DataSet colorsDataset = m_file->getDataSet(hdf5DatasetPointsPath);

  std::vector<std::vector<std::vector<float>>> colorData;
  colorsDataset.read(colorData);

  seerep_hdf5_fb::PointCloud2Iterator<uint8_t> rIter(data, offsets[0], pointStep, height, width);
  seerep_hdf5_fb::PointCloud2Iterator<uint8_t> gIter(data, offsets[1], pointStep, height, width);
  seerep_hdf5_fb::PointCloud2Iterator<uint8_t> bIter(data, offsets[2], pointStep, height, width);
  seerep_hdf5_fb::PointCloud2Iterator<uint8_t> aIter(data, offsets[3], pointStep, height, width);

  for (auto col : colorData)
  {
    for (auto row : col)
    {
      *rIter = row[0];
      *gIter = row[1];
      *bIter = row[2];
      *aIter = row[3];
      ++rIter, ++gIter, ++bIter, ++aIter;
    }
  }
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<seerep::fb::LabelWithInstance>>>
Hdf5FbPointCloud::readLabelsGeneralOffset(flatbuffers::grpc::MessageBuilder& builder, const std::string& id)
{
  std::vector<std::string> labelsGeneral;
  std::vector<std::string> labelsGeneralInstances;
  readLabelsGeneral(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD, id, labelsGeneral,
                    labelsGeneralInstances);

  std::vector<flatbuffers::Offset<seerep::fb::LabelWithInstance>> labelGeneral;
  labelGeneral.reserve(labelsGeneral.size());
  for (size_t i = 0; i < labelsGeneral.size(); i++)
  {
    auto labelOffset = builder.CreateString(labelsGeneral.at(i));
    auto instanceOffset = builder.CreateString(labelsGeneralInstances.at(i));

    seerep::fb::LabelWithInstanceBuilder labelBuilder(builder);
    labelBuilder.add_label(labelOffset);
    labelBuilder.add_instanceUuid(instanceOffset);
    labelGeneral.push_back(labelBuilder.Finish());
  }

  return builder.CreateVector<flatbuffers::Offset<seerep::fb::LabelWithInstance>>(labelGeneral);
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeled>>>
Hdf5FbPointCloud::readLabelsBoundingBoxOffset(flatbuffers::grpc::MessageBuilder& builder, const std::string& id)
{
  std::vector<std::string> boundingBoxesLabels;
  std::vector<std::vector<double>> boundingBoxes;
  std::vector<std::string> boundingBoxesInstances;
  readBoundingBoxLabeled(seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD, id, boundingBoxesLabels,
                         boundingBoxes, boundingBoxesInstances);

  std::vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeled>> boundingBoxLabeled;
  for (size_t i = 0; i < boundingBoxes.size(); i++)
  {
    auto instance = builder.CreateString(boundingBoxesInstances.at(i));
    auto label = builder.CreateString(boundingBoxesLabels.at(i));

    seerep::fb::LabelWithInstanceBuilder labelBuilder(builder);
    labelBuilder.add_instanceUuid(instance);
    labelBuilder.add_label(label);
    auto labelWithInstance = labelBuilder.Finish();

    auto pointMin = seerep::fb::CreatePoint(builder, boundingBoxes.at(i).at(0), boundingBoxes.at(i).at(1),
                                            boundingBoxes.at(i).at(2));
    auto pointMax = seerep::fb::CreatePoint(builder, boundingBoxes.at(i).at(3), boundingBoxes.at(i).at(4),
                                            boundingBoxes.at(i).at(5));

    seerep::fb::BoundingboxBuilder bbBuilder(builder);
    bbBuilder.add_point_min(pointMin);
    bbBuilder.add_point_max(pointMax);
    auto bb = bbBuilder.Finish();

    seerep::fb::BoundingBoxLabeledBuilder bblabeledBuilder(builder);
    bblabeledBuilder.add_bounding_box(bb);
    bblabeledBuilder.add_labelWithInstance(labelWithInstance);

    boundingBoxLabeled.push_back(bblabeledBuilder.Finish());
  }
  return builder.CreateVector(boundingBoxLabeled);
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
  }
  if (hasFieldx && hasFieldy && hasFieldz)
    info.has_points = true;
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
  }
  if (hasFieldx && hasFieldy && hasFieldz)
    info.has_points = true;
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
}

}  // namespace seerep_hdf5_fb
