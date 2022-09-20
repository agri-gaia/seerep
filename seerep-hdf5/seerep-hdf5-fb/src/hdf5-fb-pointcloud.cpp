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

Hdf5FbPointCloud::CloudInfo Hdf5FbPointCloud::getCloudInfo(const std::vector<std::string>& names)
{
  bool hasFieldx = false;
  bool hasFieldy = false;
  bool hasFieldz = false;

  CloudInfo info;
  for (auto fieldName : names)
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
    else
    {
      info.other_fields[fieldName] = (struct PointFieldInfo){};
    }
  }
  if (hasFieldx && hasFieldy && hasFieldz)
    info.has_points = true;
  return info;
}

// TODO read partial point cloud, e.g. only xyz without color, etc.
std::optional<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>
Hdf5FbPointCloud::readPointCloud2(const std::string& id)
{
  const std::scoped_lock lock(*m_write_mtx);

  const std::string hdf5DatasetPath = HDF5_GROUP_POINTCLOUD + "/" + id;

  // check if the point cloud exists
  if (!m_file->exist(hdf5DatasetPath))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << hdf5DatasetPath << "does not exist";
    return std::nullopt;
  }

  std::shared_ptr<HighFive::Group> data_group_ptr =
      std::make_shared<HighFive::Group>(m_file->getGroup(hdf5DatasetPath));

  flatbuffers::grpc::MessageBuilder builder;

  // read general attributes
  uint32_t height, width, point_step, row_step;
  bool is_bigendian, is_dense;

  try
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "loading general attributes";
    height = readAttributeFromHdf5<uint32_t>(id, *data_group_ptr, HEIGHT);
    width = readAttributeFromHdf5<uint32_t>(id, *data_group_ptr, WIDTH);
    point_step = readAttributeFromHdf5<uint32_t>(id, *data_group_ptr, POINT_STEP);
    row_step = readAttributeFromHdf5<uint32_t>(id, *data_group_ptr, ROW_STEP);
    is_bigendian = readAttributeFromHdf5<bool>(id, *data_group_ptr, IS_BIGENDIAN);
    is_dense = readAttributeFromHdf5<bool>(id, *data_group_ptr, IS_DENSE);
  }
  catch (const std::invalid_argument& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << "error: " << e.what();
    return std::nullopt;
  }

  // read header attributes
  auto headerAttributes = readHeaderAttributes(*data_group_ptr, id, builder);

  // read point field attributes
  std::vector<std::string> names;
  std::vector<uint32_t> offsets, counts;
  std::vector<uint8_t> datatypes;

  try
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "loading point field attributes";
    names = readAttributeFromHdf5<std::vector<std::string>>(id, *data_group_ptr, FIELD_NAME);
    offsets = readAttributeFromHdf5<std::vector<uint32_t>>(id, *data_group_ptr, FIELD_OFFSET);
    counts = readAttributeFromHdf5<std::vector<uint32_t>>(id, *data_group_ptr, FIELD_COUNT);
    datatypes = readAttributeFromHdf5<std::vector<uint8_t>>(id, *data_group_ptr, FIELD_DATATYPE);
  }
  catch (const std::invalid_argument& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << "error: " << e.what();
    return std::nullopt;
  }

  if (!sameVectorSize(names, offsets, counts, datatypes))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
        << "number of read point field attributes does not match";
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

  // combine read points fields
  CloudInfo info = getCloudInfo(names);

  std::vector<std::vector<std::vector<float>>> points;
  std::vector<std::vector<std::vector<float>>> pointsRGB;
  std::vector<std::vector<std::vector<float>>> pointsRGBA;

  if (info.has_points)
    readPoints(id, points);

  // if (info.has_rgb)
  //   readColorsRGB(id, *pointCloud2);

  // if (info.has_rgba)
  //   readColorsRGBA(id, *pointCloud2);

  // TODO normals

  // if (!info.other_fields.empty())
  //   readOtherFields(id, *pointCloud2, info.other_fields);

  // TODO extra fields other than x,y,z
  // converting the point cloud data into a 1D array
  std::vector<uint8_t> data;
  data.reserve(height * width * point_step);
  for (uint32_t row = 0; row < height; row++)
  {
    for (uint32_t col = 0; col < width; col++)
    {
      data.insert(data.end(), std::make_move_iterator(points.at(row).at(col).begin()),
                  std::make_move_iterator(points.at(row).at(col).end()));
    }
  }
  auto dataVector = builder.CreateVector(data);

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
  pointCloudBuilder.add_header(headerAttributes);
  pointCloudBuilder.add_height(height);
  pointCloudBuilder.add_width(width);
  pointCloudBuilder.add_fields(pointFieldsVector);
  pointCloudBuilder.add_is_bigendian(is_bigendian);
  pointCloudBuilder.add_point_step(point_step);
  pointCloudBuilder.add_row_step(row_step);
  pointCloudBuilder.add_data(dataVector);
  pointCloudBuilder.add_is_dense(is_dense);
  pointCloudBuilder.add_labels_general(labelsGeneralVector);
  pointCloudBuilder.add_labels_bb(boundingBoxLabeledVector);
  auto pointCloudOffset = pointCloudBuilder.Finish();
  builder.Finish(pointCloudOffset);

  auto grpcPointCloud = builder.ReleaseMessage<seerep::fb::PointCloud2>();
  return grpcPointCloud;
}

// ToDo Exception safety?
void Hdf5FbPointCloud::readPoints(const std::string& uuid, std::vector<std::vector<std::vector<float>>>& pointData)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "reading points from hdf5 file";

  std::string hdf5DatasetPointsPath =
      seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid + "/points";

  HighFive::DataSet pointsDataset = m_file->getDataSet(hdf5DatasetPointsPath);
  pointsDataset.read(pointData);
}

void Hdf5FbPointCloud::readColorsRGB(const std::string& uuid, seerep::fb::PointCloud2& cloud)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "reading rgb ";
  seerep_hdf5_fb::PointCloud2Iterator<uint8_t> r_iter(cloud, "r");
  seerep_hdf5_fb::PointCloud2Iterator<uint8_t> g_iter(cloud, "g");
  seerep_hdf5_fb::PointCloud2Iterator<uint8_t> b_iter(cloud, "b");

  std::vector<std::vector<std::vector<uint8_t>>> color_data;
  std::string colors_id = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid + "/colors";

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

void Hdf5FbPointCloud::readColorsRGBA(const std::string& uuid, seerep::fb::PointCloud2& cloud)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "reading rgb ";

  seerep_hdf5_fb::PointCloud2Iterator<uint8_t> r_iter(cloud, "r");
  seerep_hdf5_fb::PointCloud2Iterator<uint8_t> g_iter(cloud, "g");
  seerep_hdf5_fb::PointCloud2Iterator<uint8_t> b_iter(cloud, "b");
  seerep_hdf5_fb::PointCloud2Iterator<uint8_t> a_iter(cloud, "a");

  std::vector<std::vector<std::vector<uint8_t>>> color_data;
  std::string colors_id = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid + "/colors";

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

void Hdf5FbPointCloud::readOtherFields(const std::string& uuid, seerep::fb::PointCloud2& cloud,
                                       const std::map<std::string, PointFieldInfo>& fields)
{
  for (auto field_map_entry : fields)
  {
    const std::string& name = field_map_entry.first;
    const PointFieldInfo pointFieldInfo = field_map_entry.second;
    switch (pointFieldInfo.datatype)
    {
      case seerep::fb::Point_Field_Datatype_INT8:
        read<int8_t>(uuid, name, cloud, pointFieldInfo.count);
        break;
      case seerep::fb::Point_Field_Datatype_UINT8:
        read<uint8_t>(uuid, name, cloud, pointFieldInfo.count);
        break;
      case seerep::fb::Point_Field_Datatype_INT16:
        read<int16_t>(uuid, name, cloud, pointFieldInfo.count);
        break;
      case seerep::fb::Point_Field_Datatype_UINT16:
        read<uint16_t>(uuid, name, cloud, pointFieldInfo.count);
        break;
      case seerep::fb::Point_Field_Datatype_INT32:
        read<int32_t>(uuid, name, cloud, pointFieldInfo.count);
        break;
      case seerep::fb::Point_Field_Datatype_UINT32:
        read<uint32_t>(uuid, name, cloud, pointFieldInfo.count);
        break;
      case seerep::fb::Point_Field_Datatype_FLOAT32:
        read<float>(uuid, name, cloud, pointFieldInfo.count);
        break;
      case seerep::fb::Point_Field_Datatype_FLOAT64:
        read<double>(uuid, name, cloud, pointFieldInfo.count);
        break;
      default:
        std::cout << "datatype of pointcloud unknown" << std::endl;
        break;
    }
  }
}

}  // namespace seerep_hdf5_fb
