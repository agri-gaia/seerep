#include "seerep-hdf5-fb/hdf5-fb-pointcloud.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_fb
{
Hdf5FbPointCloud::Hdf5FbPointCloud(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& writeMtx)
  : Hdf5FbGeneral(file, writeMtx)
{
}

void Hdf5FbPointCloud::writePointCloud2(const std::string& id, const seerep::fb::PointCloud2& pointcloud2)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5GroupPath = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + id;
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
      << "writing flatbuffers point cloud to: " << hdf5GroupPath;

  std::shared_ptr<HighFive::Group> dataGroupPtr;

  try
  {
    checkExists(hdf5GroupPath);
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "hdf5 group" << hdf5GroupPath << " already exists!";
    dataGroupPtr = std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));
  }
  catch (std::invalid_argument const& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "hdf5 group " << hdf5GroupPath << " does not exist! Creating a new group";
    dataGroupPtr = std::make_shared<HighFive::Group>(m_file->createGroup(hdf5GroupPath));
  }

  writeAttributeToHdf5<uint32_t>(*dataGroupPtr, HEIGHT, pointcloud2.height());
  writeAttributeToHdf5<uint32_t>(*dataGroupPtr, WIDTH, pointcloud2.width());
  writeAttributeToHdf5<bool>(*dataGroupPtr, IS_BIGENDIAN, pointcloud2.is_bigendian());
  writeAttributeToHdf5<uint32_t>(*dataGroupPtr, POINT_STEP, pointcloud2.point_step());
  writeAttributeToHdf5<uint32_t>(*dataGroupPtr, ROW_STEP, pointcloud2.row_step());
  writeAttributeToHdf5<bool>(*dataGroupPtr, IS_DENSE, pointcloud2.is_dense());

  writeHeaderAttributes(*dataGroupPtr, *pointcloud2.header());

  writePointFieldAttributes(*dataGroupPtr, *pointcloud2.fields());

  if (pointcloud2.labels_bb() != nullptr)
  {
    writeBoundingBoxLabeled(HDF5_GROUP_POINTCLOUD, id, *pointcloud2.labels_bb());
  }

  if (pointcloud2.labels_general() != nullptr)
  {
    writeLabelsGeneral(HDF5_GROUP_POINTCLOUD, id, *pointcloud2.labels_general());
  }

  CloudInfo info = getCloudInfo(*pointcloud2.fields());

  if (info.has_points)
    writePoints(id, dataGroupPtr, pointcloud2);
  if (info.has_rgb)
    writeColorsRGB(id, pointcloud2);
  if (info.has_rgba)
    writeColorsRGBA(id, pointcloud2);

  // TODO normals

  // TODO other fields

  // if (!info.other_fields.empty())
  //   writeOtherFields(uuid, *pointcloud2, info.other_fields);

  m_file->flush();
}

void Hdf5FbPointCloud::writePoints(const std::string& id, const std::shared_ptr<HighFive::Group>& dataGroupPtr,
                                   const seerep::fb::PointCloud2& cloud)
{
  std::string hdf5PointsPath = HDF5_GROUP_POINTCLOUD + "/" + id + "/points";
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "writing dataset to: " << hdf5PointsPath;

  HighFive::DataSpace dataSpace({ cloud.height(), cloud.width(), 3 });
  std::shared_ptr<HighFive::DataSet> pointsDatasetPtr;

  try
  {
    checkExists(hdf5PointsPath);
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "hdf5 dataset: " << hdf5PointsPath << " already exists!";
    pointsDatasetPtr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5PointsPath));
  }
  catch (std::invalid_argument const& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "hdf5 dataset: " << hdf5PointsPath << " does not exist! Creating a new dataset";
    pointsDatasetPtr = std::make_shared<HighFive::DataSet>(m_file->createDataSet<float>(hdf5PointsPath, dataSpace));
  }

  std::vector<std::vector<std::vector<float>>> pointData;
  pointData.resize(cloud.height());

  std::array<float, 3> min, max;
  min[0] = min[1] = min[2] = std::numeric_limits<float>::max();
  max[0] = max[1] = max[2] = std::numeric_limits<float>::min();

  seerep_hdf5_fb::PointCloud2ReadIterator<float> xIter(cloud, "x");
  seerep_hdf5_fb::PointCloud2ReadIterator<float> yIter(cloud, "y");
  seerep_hdf5_fb::PointCloud2ReadIterator<float> zIter(cloud, "z");

  for (size_t row = 0; row < cloud.height(); row++)
  {
    pointData[row].reserve(cloud.width());
    for (size_t col = 0; col < cloud.width(); col++)
    {
      const float& x = *xIter;
      const float& y = *yIter;
      const float& z = *zIter;

      // compute bounding box for indicies at the same time
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

      pointData[row].push_back(std::vector{ x, y, z });

      ++xIter, ++yIter, ++zIter;
    }
  }

  // min (x, y, z), max (x,y,z)
  const std::vector boundingBox{ min[0], min[1], min[2], max[0], max[1], max[2] };

  // write bounding box as data group attribute
  writeAttributeToHdf5(*dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::BOUNDINGBOX, boundingBox);

  // write points to dataset
  pointsDatasetPtr->write(pointData);
}

void Hdf5FbPointCloud::writeColorsRGB(const std::string& uuid, const seerep::fb::PointCloud2& cloud)
{
  const std::string hdf5ColorsPath = HDF5_GROUP_POINTCLOUD + "/" + uuid + "/colors";
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "writing dataset to: " << hdf5ColorsPath;

  HighFive::DataSpace dataSpace({ cloud.height(), cloud.width(), 3 });
  std::shared_ptr<HighFive::DataSet> colorsDatasetPtr;

  try
  {
    checkExists(hdf5ColorsPath);
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "hdf5 dataset: " << hdf5ColorsPath << " already exists!";
    colorsDatasetPtr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5ColorsPath));
  }
  catch (std::invalid_argument const& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "hdf5 dataset: " << hdf5ColorsPath << " does not exist! Creating a new dataset";
    colorsDatasetPtr = std::make_shared<HighFive::DataSet>(m_file->createDataSet<float>(hdf5ColorsPath, dataSpace));
  }

  std::vector<std::vector<std::vector<uint8_t>>> colorsData;
  colorsData.resize(cloud.height());

  seerep_hdf5_fb::PointCloud2ReadIterator<uint8_t> rIter(cloud, "r");
  seerep_hdf5_fb::PointCloud2ReadIterator<uint8_t> gIter(cloud, "g");
  seerep_hdf5_fb::PointCloud2ReadIterator<uint8_t> bIter(cloud, "b");

  for (size_t row = 0; row < cloud.height(); row++)
  {
    colorsData[row].reserve(cloud.width());
    for (size_t col = 0; col < cloud.width(); col++)
    {
      colorsData[row].push_back(std::vector{ *rIter, *gIter, *bIter });
      ++rIter, ++gIter, ++bIter;
    }
  }

  colorsDatasetPtr->write(colorsData);
}

void Hdf5FbPointCloud::writeColorsRGBA(const std::string& uuid, const seerep::fb::PointCloud2& cloud)
{
  const std::string hdf5ColorsPath = HDF5_GROUP_POINTCLOUD + "/" + uuid + "/colors";
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "writing dataset to: " << hdf5ColorsPath;

  HighFive::DataSpace dataSpace({ cloud.height(), cloud.width(), 4 });

  std::shared_ptr<HighFive::DataSet> colorsDatasetPtr;
  try
  {
    checkExists(hdf5ColorsPath);
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "hdf5 dataset: " << hdf5ColorsPath << " already exists!";
    colorsDatasetPtr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5ColorsPath));
  }
  catch (std::invalid_argument const& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "hdf5 dataset: " << hdf5ColorsPath << " does not exist! Creating a new dataset";
    colorsDatasetPtr = std::make_shared<HighFive::DataSet>(m_file->createDataSet<float>(hdf5ColorsPath, dataSpace));
  }

  std::vector<std::vector<std::vector<uint8_t>>> colorsData;
  colorsData.resize(cloud.height());

  seerep_hdf5_fb::PointCloud2ReadIterator<uint8_t> rIter(cloud, "r");
  seerep_hdf5_fb::PointCloud2ReadIterator<uint8_t> gIter(cloud, "g");
  seerep_hdf5_fb::PointCloud2ReadIterator<uint8_t> bIter(cloud, "b");
  seerep_hdf5_fb::PointCloud2ReadIterator<uint8_t> aIter(cloud, "a");

  for (size_t row = 0; row < cloud.height(); row++)
  {
    colorsData[row].reserve(cloud.width());
    for (size_t col = 0; col < cloud.width(); col++)
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

Hdf5FbPointCloud::CloudInfo
Hdf5FbPointCloud::getCloudInfo(const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::PointField>>& pointFields)
{
  bool hasFieldx = false;
  bool hasFieldy = false;
  bool hasFieldz = false;

  CloudInfo info;
  for (size_t i = 0; i < pointFields.size(); i++)
  {
    const std::string& fieldName = pointFields.Get(i)->name()->str();
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
      info.other_fields[fieldName] = (struct PointFieldInfo){};
    }
  }
  if (hasFieldx && hasFieldy && hasFieldz)
    info.has_points = true;
  return info;
}

// Assumption: names and offsets vector have the same order of the fields
uint32_t Hdf5FbPointCloud::getOffset(const std::vector<std::string>& names, const std::vector<uint32_t>& offsets,
                                     const std::string& fieldName, bool isBigendian, uint32_t poinStep)
{
  for (size_t i = 0; i < names.size(); i++)
  {
    if (names[i] == fieldName)
    {
      return offsets[i];
    }
  }

  // Handle the special case of rgb(a), since the rgb(a) channel is encoded in a single 32 Bit integer

  // Needs C++17
  std::set<std::string_view> fieldNames = { "r", "g", "b", "a" };

  if (fieldNames.find(fieldName) != fieldNames.end())
  {
    for (size_t i = 0; i < names.size(); i++)
    {
      if (names[i] == "rgb" || names[i] == "rgba")
      {
        if (fieldName == "r")
        {
          if (isBigendian)
          {
            return offsets[i] + 1;
          }
          else
          {
            return offsets[i] + 2;
          }
        }
        if (fieldName == "g")
        {
          if (isBigendian)
          {
            return offsets[i] + 2;
          }
          else
          {
            return offsets[i] + 1;
          }
        }
        if (fieldName == "b")
        {
          if (isBigendian)
          {
            return offsets[i] + 3;
          }
          else
          {
            return offsets[i] + 0;
          }
        }
        if (fieldName == "a")
        {
          if (isBigendian)
          {
            return offsets[i] + 0;
          }
          else
          {
            return offsets[i] + 3;
          }
        }
      }
    }
  }
  throw std::runtime_error("Field " + fieldName + " does not exist!");
}

// TODO read partial point cloud, e.g. only xyz without color, etc.
std::optional<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>
Hdf5FbPointCloud::readPointCloud2(const std::string& id)
{
  const std::scoped_lock lock(*m_write_mtx);

  const std::string hdf5GroupPath = HDF5_GROUP_POINTCLOUD + "/" + id;
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
    height = readAttributeFromHdf5<uint32_t>(id, *dataGroupPtr, HEIGHT);
    width = readAttributeFromHdf5<uint32_t>(id, *dataGroupPtr, WIDTH);
    pointStep = readAttributeFromHdf5<uint32_t>(id, *dataGroupPtr, POINT_STEP);
    rowStep = readAttributeFromHdf5<uint32_t>(id, *dataGroupPtr, ROW_STEP);
    isBigendian = readAttributeFromHdf5<bool>(id, *dataGroupPtr, IS_BIGENDIAN);
    isDense = readAttributeFromHdf5<bool>(id, *dataGroupPtr, IS_DENSE);
  }
  catch (const std::invalid_argument& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "error loading general attributes of: " << hdf5GroupPath;
    return std::nullopt;
  }

  // reads and creates a flatbuffers header with offset
  auto headerAttributesOffset = readHeaderAttributes(builder, *dataGroupPtr, id);

  // read point field attributes
  std::vector<std::string> names;
  std::vector<uint32_t> offsets, counts;
  std::vector<uint8_t> datatypes;

  try
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
        << "loading point field attributes of: " << hdf5GroupPath;
    names = readAttributeFromHdf5<std::vector<std::string>>(id, *dataGroupPtr, FIELD_NAME);
    offsets = readAttributeFromHdf5<std::vector<uint32_t>>(id, *dataGroupPtr, FIELD_OFFSET);
    counts = readAttributeFromHdf5<std::vector<uint32_t>>(id, *dataGroupPtr, FIELD_COUNT);
    datatypes = readAttributeFromHdf5<std::vector<uint8_t>>(id, *dataGroupPtr, FIELD_DATATYPE);
  }
  catch (const std::invalid_argument& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "error loading point field attributes of: " << hdf5GroupPath;
    return std::nullopt;
  }

  if (!equalVectorLength(names, offsets, counts, datatypes))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "unequal number of point field attributes: " << hdf5GroupPath;
    return std::nullopt;
  }

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
  auto pointFieldsVector = builder.CreateVector(pointFields);

  CloudInfo info = getCloudInfo(*reinterpret_cast<flatbuffers::Vector<flatbuffers::Offset<seerep::fb::PointField>>*>(
      builder.GetCurrentBufferPointer()));

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
      << "reading point cloud data of: " << hdf5GroupPath;

  // pre allocate vector, to write values in it from the iterators
  // should be way faster then copying it in vectors;

  // pointer to the data field of the pre-allocated array
  uint8_t* data;
  // allocate height * width * point setp bytes
  auto vector = builder.CreateUninitializedVector(height * width * pointStep, sizeof(uint8_t), &data);

  if (info.has_points)
  {
    const std::string hdf5DatasetPointsPath = HDF5_GROUP_POINTCLOUD + "/" + id + "/points";

    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
        << "reading points from: " << hdf5DatasetPointsPath;

    HighFive::DataSet pointsDataset = m_file->getDataSet(hdf5DatasetPointsPath);

    std::vector<std::vector<std::vector<float>>> pointData;
    pointsDataset.read(pointData);

    int xOffset = getOffset(names, offsets, "x", isBigendian, pointStep);
    int yOffset = getOffset(names, offsets, "y", isBigendian, pointStep);
    int zOffset = getOffset(names, offsets, "z", isBigendian, pointStep);

    seerep_hdf5_fb::PointCloud2WriteIterator<float> xIter(data, xOffset, height * width, pointStep);
    seerep_hdf5_fb::PointCloud2WriteIterator<float> yIter(data, yOffset, height * width, pointStep);
    seerep_hdf5_fb::PointCloud2WriteIterator<float> zIter(data, zOffset, height * width, pointStep);

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

  if (info.has_rgb)
  {
    const std::string hdf5DatasetColorsPath = HDF5_GROUP_POINTCLOUD + "/" + id + "/colors";

    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
        << "reading rgb from: " << hdf5DatasetColorsPath;

    HighFive::DataSet colorsDataset = m_file->getDataSet(hdf5DatasetColorsPath);

    std::vector<std::vector<std::vector<uint8_t>>> colorData;
    colorsDataset.read(colorData);

    int rOffset = getOffset(names, offsets, "r", isBigendian, pointStep);
    int gOffset = getOffset(names, offsets, "g", isBigendian, pointStep);
    int bOffset = getOffset(names, offsets, "b", isBigendian, pointStep);

    seerep_hdf5_fb::PointCloud2WriteIterator<uint8_t> rIter(data, rOffset, height * width, pointStep);
    seerep_hdf5_fb::PointCloud2WriteIterator<uint8_t> gIter(data, gOffset, height * width, pointStep);
    seerep_hdf5_fb::PointCloud2WriteIterator<uint8_t> bIter(data, bOffset, height * width, pointStep);

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

  if (info.has_rgba)
  {
    const std::string hdf5DatasetColorsPath = HDF5_GROUP_POINTCLOUD + "/" + id + "/colors";

    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
        << "reading rgba from: " << hdf5DatasetColorsPath;

    HighFive::DataSet colorsDataset = m_file->getDataSet(hdf5DatasetColorsPath);

    std::vector<std::vector<std::vector<uint8_t>>> colorData;
    colorsDataset.read(colorData);

    int rOffset = getOffset(names, offsets, "r", isBigendian, pointStep);
    int gOffset = getOffset(names, offsets, "g", isBigendian, pointStep);
    int bOffset = getOffset(names, offsets, "b", isBigendian, pointStep);
    int aOffset = getOffset(names, offsets, "a", isBigendian, pointStep);

    seerep_hdf5_fb::PointCloud2WriteIterator<uint8_t> rIter(data, rOffset, height * width, pointStep);
    seerep_hdf5_fb::PointCloud2WriteIterator<uint8_t> gIter(data, gOffset, height * width, pointStep);
    seerep_hdf5_fb::PointCloud2WriteIterator<uint8_t> bIter(data, bOffset, height * width, pointStep);
    seerep_hdf5_fb::PointCloud2WriteIterator<uint8_t> aIter(data, aOffset, height * width, pointStep);

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

  // // TODO add other fields

  // // if (!info.other_fields.empty())
  // //   readOtherFields(id, info.other_fields, otherFields);

  // // TODO add normals

  // auto dataVector = builder.CreateVector(data);

  // read labeled bounding box
  std::vector<std::string> boundingBoxesLabels;
  std::vector<std::vector<double>> boundingBoxes;
  std::vector<std::string> boundingBoxesInstances;
  readBoundingBoxLabeled(HDF5_GROUP_POINTCLOUD, id, boundingBoxesLabels, boundingBoxes, boundingBoxesInstances);

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
  auto boundingBoxLabeledVector = builder.CreateVector(boundingBoxLabeled);

  // read labels general
  std::vector<std::string> labelsGeneral;
  std::vector<std::string> labelsGeneralInstances;
  readLabelsGeneral(HDF5_GROUP_POINTCLOUD, id, labelsGeneral, labelsGeneralInstances);

  // TODO add this to a higher level, it's the same for images and point clouds
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

  auto labelsGeneralVector = builder.CreateVector<flatbuffers::Offset<seerep::fb::LabelWithInstance>>(labelGeneral);

  // build the point cloud message
  seerep::fb::PointCloud2Builder pointCloudBuilder(builder);
  pointCloudBuilder.add_header(headerAttributesOffset);
  pointCloudBuilder.add_height(height);
  pointCloudBuilder.add_width(width);
  pointCloudBuilder.add_fields(pointFieldsVector);
  pointCloudBuilder.add_is_bigendian(isBigendian);
  pointCloudBuilder.add_point_step(pointStep);
  pointCloudBuilder.add_row_step(rowStep);
  pointCloudBuilder.add_data(vector);
  pointCloudBuilder.add_is_dense(isDense);
  pointCloudBuilder.add_labels_general(labelsGeneralVector);
  pointCloudBuilder.add_labels_bb(boundingBoxLabeledVector);
  auto pointCloudOffset = pointCloudBuilder.Finish();
  builder.Finish(pointCloudOffset);

  auto grpcPointCloud = builder.ReleaseMessage<seerep::fb::PointCloud2>();
  return grpcPointCloud;
}

// TODO check if the copy into the tmp vector can be removed somehow
void Hdf5FbPointCloud::readPoints(const std::string& uuid, uint8_t* data)
{
}

void Hdf5FbPointCloud::readColorsRGB(const std::string& uuid, std::vector<std::vector<std::vector<uint8_t>>>& colorData)
{
}

void Hdf5FbPointCloud::readColorsRGBA(const std::string& uuid, std::vector<std::vector<std::vector<uint8_t>>>& colorData)
{
  const std::string hdf5DatasetColorsPath = HDF5_GROUP_POINTCLOUD + "/" + uuid + "/colors";

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "reading rgba from: " << hdf5DatasetColorsPath;

  HighFive::DataSet colorsDataset = m_file->getDataSet(hdf5DatasetColorsPath);

  colorsDataset.read(colorData);
}

void Hdf5FbPointCloud::readOtherFields(const std::string& uuid, const std::map<std::string, PointFieldInfo>& fields,
                                       std::vector<std::any> otherFieldsData)
{
  for (auto field_map_entry : fields)
  {
    const std::string& name = field_map_entry.first;
    const PointFieldInfo pointFieldInfo = field_map_entry.second;
    switch (pointFieldInfo.datatype)
    {
      case seerep::fb::Point_Field_Datatype_INT8:
      {
        std::vector<int8_t> data;
        read<int8_t>(uuid, name, data, pointFieldInfo.count);
        otherFieldsData.push_back(std::move(data));
      }
      break;
      case seerep::fb::Point_Field_Datatype_UINT8:
      {
        std::vector<uint8_t> data;
        read<uint8_t>(uuid, name, data, pointFieldInfo.count);
        otherFieldsData.push_back(std::move(data));
      }
      break;
      case seerep::fb::Point_Field_Datatype_INT16:
      {
        std::vector<int16_t> data;
        read<int16_t>(uuid, name, data, pointFieldInfo.count);
        otherFieldsData.push_back(std::move(data));
      }
      break;
      case seerep::fb::Point_Field_Datatype_UINT16:
      {
        std::vector<uint16_t> data;
        read<uint16_t>(uuid, name, data, pointFieldInfo.count);
        otherFieldsData.push_back(std::move(data));
      }
      break;
      case seerep::fb::Point_Field_Datatype_INT32:
      {
        std::vector<int32_t> data;
        read<int32_t>(uuid, name, data, pointFieldInfo.count);
        otherFieldsData.push_back(std::move(data));
      }
      break;
      case seerep::fb::Point_Field_Datatype_UINT32:
      {
        std::vector<uint32_t> data;
        read<uint32_t>(uuid, name, data, pointFieldInfo.count);
        otherFieldsData.push_back(std::move(data));
      }
      break;
      case seerep::fb::Point_Field_Datatype_FLOAT32:
      {
        std::vector<float> data;
        read<float>(uuid, name, data, pointFieldInfo.count);
        otherFieldsData.push_back(std::move(data));
      }
      break;
      case seerep::fb::Point_Field_Datatype_FLOAT64:
      {
        std::vector<double> data;
        read<double>(uuid, name, data, pointFieldInfo.count);
        otherFieldsData.push_back(std::move(data));
      }
      break;
      default:
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
            << "datatype of field in point cloud unknown";
        break;
    }
  }
}

}  // namespace seerep_hdf5_fb
