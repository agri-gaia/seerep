#include "seerep_hdf5_core/hdf5_core_image.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_core
{
Hdf5CoreImage::Hdf5CoreImage(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx)
{
}

std::optional<seerep_core_msgs::DatasetIndexable> Hdf5CoreImage::readDataset(const boost::uuids::uuid& uuid)
{
  return readDataset(boost::lexical_cast<std::string>(uuid));
}

std::optional<seerep_core_msgs::DatasetIndexable> Hdf5CoreImage::readDataset(const std::string& uuid)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5DataGroupPath = getHdf5GroupPath(uuid);

  auto dataGroupPtr = getHdf5Group(hdf5DataGroupPath);

  if (!dataGroupPtr)
  {
    return std::nullopt;
  }

  seerep_core_msgs::DatasetIndexable data;

  data.header.datatype = seerep_core_msgs::Datatype::Image;

  data.header.frameId = readAttributeFromHdf5<std::string>(hdf5DataGroupPath, *dataGroupPtr,
                                                           seerep_hdf5_core::Hdf5CoreImage::HEADER_FRAME_ID);
  data.header.timestamp.seconds = readAttributeFromHdf5<int64_t>(hdf5DataGroupPath, *dataGroupPtr,
                                                                 seerep_hdf5_core::Hdf5CoreImage::HEADER_STAMP_SECONDS);
  data.header.timestamp.nanos = readAttributeFromHdf5<int64_t>(hdf5DataGroupPath, *dataGroupPtr,
                                                               seerep_hdf5_core::Hdf5CoreImage::HEADER_STAMP_NANOS);

  boost::uuids::string_generator gen;
  boost::uuids::uuid uuid_generated = gen(uuid);
  data.header.uuidData = uuid_generated;

  // set bounding box for images to 0. assume no spatial extent
  data.boundingbox.min_corner().set<0>(0);
  data.boundingbox.min_corner().set<1>(0);
  data.boundingbox.min_corner().set<2>(0);
  data.boundingbox.max_corner().set<0>(0);
  data.boundingbox.max_corner().set<1>(0);
  data.boundingbox.max_corner().set<2>(0);

  readLabelsGeneralAndAddToLabelsWithInstancesWithCategory(HDF5_GROUP_IMAGE, uuid, data.labelsWithInstancesWithCategory);

  readBoundingBoxLabeledAndAddToLabelsWithInstancesWithCategory(HDF5_GROUP_IMAGE, uuid,
                                                                data.labelsWithInstancesWithCategory);

  return data;
}

std::vector<std::string> Hdf5CoreImage::getDatasetUuids()
{
  return getGroupDatasets(HDF5_GROUP_IMAGE);
}

void Hdf5CoreImage::writeLabelsGeneral(
    const std::string& uuid,
    const std::vector<seerep_core_msgs::LabelsWithInstanceWithCategory>& labelsWithInstanceWithCategory)
{
  Hdf5CoreGeneral::writeLabelsGeneral(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, uuid,
                                      labelsWithInstanceWithCategory);
}

void Hdf5CoreImage::writeImageAttributes(const std::string& id, const ImageAttributes& attributes)
{
  std::string hdf5GroupPath = getHdf5GroupPath(id);
  std::string hdf5DatasetPath = getHdf5DataSetPath(id);

  auto dataGroupPtr = getHdf5Group(hdf5GroupPath, false);
  auto dataSetPtr = getHdf5DataSet(hdf5DatasetPath);

  if (dataGroupPtr && dataSetPtr)
  {
    writeAttributeToHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CoreImage::HEIGHT, attributes.height);
    writeAttributeToHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CoreImage::WIDTH, attributes.width);
    writeAttributeToHdf5<std::string>(*dataSetPtr, seerep_hdf5_core::Hdf5CoreImage::ENCODING, attributes.encoding);
    writeAttributeToHdf5<bool>(*dataSetPtr, seerep_hdf5_core::Hdf5CoreImage::IS_BIGENDIAN, attributes.isBigendian);
    writeAttributeToHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CoreImage::STEP, attributes.step);
  }
}

ImageAttributes Hdf5CoreImage::readImageAttributes(const std::string& id)
{
  std::string hdf5GroupPath = getHdf5GroupPath(id);
  std::string hdf5DatasetPath = getHdf5DataSetPath(id);

  auto dataGroupPtr = getHdf5Group(hdf5GroupPath, false);
  auto dataSetPtr = getHdf5DataSet(hdf5DatasetPath);

  ImageAttributes attributes;

  if (dataGroupPtr && dataSetPtr)
  {
    attributes.height = readAttributeFromHdf5<uint32_t>(id, *dataGroupPtr, seerep_hdf5_core::Hdf5CoreImage::HEIGHT);
    attributes.width = readAttributeFromHdf5<uint32_t>(id, *dataGroupPtr, seerep_hdf5_core::Hdf5CoreImage::WIDTH);
    attributes.encoding =
        readAttributeFromHdf5<std::string>(id, *dataSetPtr, seerep_hdf5_core::Hdf5CoreImage::ENCODING);
    attributes.isBigendian =
        readAttributeFromHdf5<bool>(id, *dataSetPtr, seerep_hdf5_core::Hdf5CoreImage::IS_BIGENDIAN);
    attributes.step = readAttributeFromHdf5<uint32_t>(id, *dataGroupPtr, seerep_hdf5_core::Hdf5CoreImage::STEP);
  }
  return attributes;
}

const std::string Hdf5CoreImage::getHdf5GroupPath(const std::string& id) const
{
  return HDF5_GROUP_IMAGE + "/" + id;
}

const std::string Hdf5CoreImage::getHdf5DataSetPath(const std::string& id) const
{
  return getHdf5GroupPath(id) + "/" + RAWDATA;
}
}  // namespace seerep_hdf5_core
