#include "seerep-hdf5-core/hdf5-core-point-cloud.h"

namespace seerep_hdf5_core
{
Hdf5CorePointCloud::Hdf5CorePointCloud(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx)
{
}

std::optional<seerep_core_msgs::DatasetIndexable> Hdf5CorePointCloud::readDataset(const boost::uuids::uuid& uuid)
{
  return readDataset(boost::lexical_cast<std::string>(uuid));
}

std::optional<seerep_core_msgs::DatasetIndexable> Hdf5CorePointCloud::readDataset(const std::string& uuid)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5DatasetPath = HDF5_GROUP_POINTCLOUD + "/" + uuid;

  if (!m_file->exist(hdf5DatasetPath))
  {
    return std::nullopt;
  }

  std::shared_ptr<HighFive::Group> group_ptr = std::make_shared<HighFive::Group>(m_file->getGroup(hdf5DatasetPath));

  seerep_core_msgs::DatasetIndexable data;

  data.header.datatype = seerep_core_msgs::Datatype::PointCloud;

  boost::uuids::string_generator gen;
  data.header.uuidData = gen(uuid);

  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::HEADER_FRAME_ID).read(data.header.frameId);
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::HEADER_STAMP_SECONDS).read(data.header.timestamp.seconds);
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::HEADER_STAMP_NANOS).read(data.header.timestamp.nanos);

  std::vector<float> bb;
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::BOUNDINGBOX).read(bb);
  data.boundingbox.min_corner().set<0>(bb.at(0));
  data.boundingbox.min_corner().set<1>(bb.at(1));
  data.boundingbox.min_corner().set<2>(bb.at(2));
  data.boundingbox.max_corner().set<0>(bb.at(3));
  data.boundingbox.max_corner().set<1>(bb.at(4));
  data.boundingbox.max_corner().set<2>(bb.at(5));

  readLabelsGeneralAndAddToLabelsWithInstancesWithCategory(HDF5_GROUP_POINTCLOUD, uuid,
                                                           data.labelsWithInstancesWithCategory);

  readBoundingBoxLabeledAndAddToLabelsWithInstancesWithCategory(HDF5_GROUP_POINTCLOUD, uuid,
                                                                data.labelsWithInstancesWithCategory);

  // std::vector<std::string> labelCategoriesBB;
  // std::vector<std::vector<std::string>> labelsBBPerCategory;
  // std::vector<std::vector<std::vector<double>>> boundingBoxesPerCategory;
  // std::vector<std::vector<std::string>> instancesPerCategory;
  // readBoundingBoxLabeled(HDF5_GROUP_POINTCLOUD, uuid, labelCategoriesBB, labelsBBPerCategory, boundingBoxesPerCategory,
  //                        instancesPerCategory, false);

  // // loop the label categories
  // for (std::size_t i = 0; i < labelCategoriesBB.size(); i++)
  // {
  //   auto& labelsBB = labelsBBPerCategory.at(i);
  //   auto& instances = instancesPerCategory.at(i);

  //   // check if category already exists in data
  //   // create new one if it doesn't exist
  //   auto labelsWithInstanceOfCategory = data.labelsWithInstancesWithCategory.find(labelCategoriesBB.at(i));
  //   if (labelsWithInstanceOfCategory == data.labelsWithInstancesWithCategory.end())
  //   {
  //     auto emplaceResult = data.labelsWithInstancesWithCategory.emplace(
  //         labelCategoriesBB.at(i), std::vector<seerep_core_msgs::LabelWithInstance>());
  //     labelsWithInstanceOfCategory = emplaceResult.first;
  //   }

  //   // add labels with instance to this label category
  //   for (std::size_t i = 0; i < labelsBB.size(); i++)
  //   {
  //     boost::uuids::uuid instanceUuid;
  //     try
  //     {
  //       instanceUuid = gen(instances.at(i));
  //     }
  //     catch (std::runtime_error&)
  //     {
  //       instanceUuid = boost::uuids::nil_uuid();
  //     }
  //     labelsWithInstanceOfCategory->second.push_back(
  //         seerep_core_msgs::LabelWithInstance{ .label = labelsBB.at(i), .uuidInstance = instanceUuid });
  //   }
  // }
  return data;
}

std::vector<std::string> Hdf5CorePointCloud::getDatasetUuids()
{
  return getGroupDatasets(HDF5_GROUP_POINTCLOUD);
}

std::vector<float> Hdf5CorePointCloud::writePoints(const std::string& plcUUID, PCLIterInfos infos)
{
  const std::string PCLGroupPath = seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + plcUUID;
  const std::string pointsPath = PCLGroupPath + "/points";

  HighFive::DataSpace dataSpace({ infos.height, infos.width, 3 });
  std::shared_ptr<HighFive::DataSet> pointsDatasetPtr = getHdf5DataSet<float>(pointsPath, dataSpace);

  std::vector<std::vector<std::vector<float>>> pointData;
  pointData.resize(infos.height);

  std::array<float, 3> min, max;
  min[0] = min[1] = min[2] = std::numeric_limits<float>::max();
  max[0] = max[1] = max[2] = std::numeric_limits<float>::min();

  seerep_hdf5_core::PointCloud2ConstIterator<float> xIter(infos.data, infos.offsets["x"], infos.pointStep, infos.height,
                                                          infos.width);
  seerep_hdf5_core::PointCloud2ConstIterator<float> yIter(infos.data, infos.offsets["y"], infos.pointStep, infos.height,
                                                          infos.width);
  seerep_hdf5_core::PointCloud2ConstIterator<float> zIter(infos.data, infos.offsets["z"], infos.pointStep, infos.height,
                                                          infos.width);

  for (size_t row = 0; row < infos.height; row++)
  {
    pointData[row].reserve(infos.width);
    for (size_t col = 0; col < infos.width; col++)
    {
      const float& x = *xIter;
      const float& y = *yIter;
      const float& z = *zIter;

      updateBoundingBox(min, max, x, y, z);

      pointData[row].push_back(std::vector{ x, y, z });

      ++xIter, ++yIter, ++zIter;
    }
  }

  std::vector<float> bb({ min[0], min[1], min[2], max[0], max[1], max[2] });
  std::shared_ptr<HighFive::Group> group = getHdf5Group(PCLGroupPath);
  writeAttributeToHdf5<std::vector<float>>(*group, BOUNDINGBOX, bb);

  pointsDatasetPtr->write(pointData);
  return bb;
}

// TODO can be unified with RGBA write method
void Hdf5CorePointCloud::writeRGB(const std::string& pclUUID, PCLIterInfos infos)
{
  const std::string hdf5ColorsPath =
      seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + pclUUID + "/colors";

  HighFive::DataSpace dataSpace({ infos.height, infos.width, 3 });
  std::shared_ptr<HighFive::DataSet> colorsDatasetPtr = getHdf5DataSet<float>(hdf5ColorsPath, dataSpace);

  std::vector<std::vector<std::vector<uint8_t>>> colorsData;
  colorsData.resize(infos.height);

  seerep_hdf5_core::PointCloud2ConstIterator<uint8_t> rIter(infos.data, infos.offsets["r"], infos.pointStep,
                                                            infos.height, infos.width);

  seerep_hdf5_core::PointCloud2ConstIterator<uint8_t> gIter(infos.data, infos.offsets["g"], infos.pointStep,
                                                            infos.height, infos.width);
  seerep_hdf5_core::PointCloud2ConstIterator<uint8_t> bIter(infos.data, infos.offsets["b"], infos.pointStep,
                                                            infos.height, infos.width);
  for (size_t row = 0; row < infos.height; row++)
  {
    colorsData[row].reserve(infos.width);
    for (size_t col = 0; col < infos.width; col++)
    {
      colorsData[row].push_back(std::vector{ *rIter, *gIter, *bIter });
      ++rIter, ++gIter, ++bIter;
    }
  }

  colorsDatasetPtr->write(colorsData);
}

void Hdf5CorePointCloud::writeRGBA(const std::string& pclUUID, PCLIterInfos infos)
{
  const std::string hdf5ColorsPath =
      seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + pclUUID + "/colors";

  HighFive::DataSpace dataSpace({ infos.height, infos.width, 4 });
  std::shared_ptr<HighFive::DataSet> colorsDatasetPtr = getHdf5DataSet<float>(hdf5ColorsPath, dataSpace);

  std::vector<std::vector<std::vector<uint8_t>>> colorsData;
  colorsData.resize(infos.height);

  seerep_hdf5_core::PointCloud2ConstIterator<uint8_t> rIter(infos.data, infos.offsets["r"], infos.pointStep,
                                                            infos.height, infos.width);

  seerep_hdf5_core::PointCloud2ConstIterator<uint8_t> gIter(infos.data, infos.offsets["g"], infos.pointStep,
                                                            infos.height, infos.width);
  seerep_hdf5_core::PointCloud2ConstIterator<uint8_t> bIter(infos.data, infos.offsets["b"], infos.pointStep,
                                                            infos.height, infos.width);
  seerep_hdf5_core::PointCloud2ConstIterator<uint8_t> aIter(infos.data, infos.offsets["a"], infos.pointStep,
                                                            infos.height, infos.width);

  for (size_t row = 0; row < infos.height; row++)
  {
    colorsData[row].reserve(infos.width);
    for (size_t col = 0; col < infos.width; col++)
    {
      colorsData[row].push_back(std::vector{ *rIter, *gIter, *bIter, *aIter });
      ++rIter, ++gIter, ++bIter;
      ++aIter;
    }
  }

  colorsDatasetPtr->write(colorsData);
}

void Hdf5CorePointCloud::updateBoundingBox(std::array<float, 3>& min, std::array<float, 3>& max, const float& x,
                                           const float& y, const float& z)
{
  if (x < min[0])
  {
    min[0] = x;
  }
  if (x > max[0])
  {
    max[0] = x;
  }

  if (y < min[1])
  {
    min[1] = y;
  }
  if (y > max[1])
  {
    max[1] = y;
  }

  if (z < min[2])
  {
    min[2] = z;
  }
  if (z > max[2])
  {
    max[2] = z;
  }
}

PCLChannels Hdf5CorePointCloud::getChannels(const std::vector<std::string>& fields)
{
  bool hasFieldx = false;
  bool hasFieldy = false;
  bool hasFieldz = false;

  PCLChannels channels;

  for (const std::string& fieldName : fields)
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
      channels.has_rgb = true;
    }
    else if (fieldName == "rgba")
    {
      channels.has_rgba = true;
    }
    else if (fieldName.find("normal") == 0)
    {
      channels.has_normals = true;
    }
  }
  if (hasFieldx && hasFieldy && hasFieldz)
  {
    channels.has_points = true;
  }
  return channels;
}

uint32_t Hdf5CorePointCloud::getRgbaOffset(const std::string& fieldName, uint32_t offset, bool isBigendian)
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

void Hdf5CorePointCloud::writePCL(const std::string& pclUUID, PCLChannels channels, PCLIterInfos iterInfos)
{
  if (channels.has_points)
  {
    writePoints(pclUUID, iterInfos);
  }
  else if (channels.has_rgb)
  {
    writeRGB(pclUUID, iterInfos);
  }
  else if (channels.has_rgba)
  {
    writeRGBA(pclUUID, iterInfos);
  }
  else if (channels.has_normals)
  {
    // TODO
  }

  // TODO other fields
}

std::map<std::string, uint32_t> Hdf5CorePointCloud::getOffsets(const std::vector<uint32_t>& offsets,
                                                               const std::vector<std::string>& fieldNames,
                                                               bool isBigendian)
{
  if (offsets.size() != fieldNames.size())
  {
    throw std::runtime_error("Different number of offsets and field names");
  }

  std::map<std::string, uint32_t> mapOffsets;
  for (size_t i = 0; i < fieldNames.size(); i++)
  {
    if (fieldNames[i] == "rgb")
    {
      std::set rgbSet = { "r", "g", "b" };
      for (const std::string& field : rgbSet)
      {
        mapOffsets[field] = getRgbaOffset(field, offsets[i], isBigendian);
      }
    }
    else if (fieldNames[i] == "rgba")
    {
      std::set rgbaSet = { "r", "g", "b", "a" };
      for (const std::string& field : rgbaSet)
      {
        mapOffsets[field] = getRgbaOffset(field, offsets[i], isBigendian);
      }
    }
    else
    {
      mapOffsets[fieldNames[i]] = offsets[i];
    }
  }
  return mapOffsets;
}

}  // namespace seerep_hdf5_core
