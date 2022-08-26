#include "seerep-hdf5-core/hdf5-core-image.h"

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

  std::string hdf5DatasetPath = HDF5_GROUP_IMAGE + "/" + uuid;
  std::string hdf5DatasetRawDataPath = hdf5DatasetPath + "/" + seerep_hdf5_core::Hdf5CoreImage::RAWDATA;

  if (!m_file->exist(hdf5DatasetPath) || !m_file->exist(hdf5DatasetRawDataPath))
    return std::nullopt;

  std::shared_ptr<HighFive::DataSet> data_set_ptr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRawDataPath));

  seerep_core_msgs::DatasetIndexable data;

  data.header.frameId = readAttributeFromHdf5<std::string>(hdf5DatasetRawDataPath, *data_set_ptr,
                                                           seerep_hdf5_core::Hdf5CoreImage::HEADER_FRAME_ID);
  data.header.timestamp.seconds = readAttributeFromHdf5<int64_t>(hdf5DatasetRawDataPath, *data_set_ptr,
                                                                 seerep_hdf5_core::Hdf5CoreImage::HEADER_STAMP_SECONDS);
  data.header.timestamp.nanos = readAttributeFromHdf5<int64_t>(hdf5DatasetRawDataPath, *data_set_ptr,
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

  std::vector<std::string> labelsGeneral;
  std::vector<std::string> instancesGeneral;
  readLabelsGeneral(HDF5_GROUP_IMAGE, uuid, labelsGeneral, instancesGeneral);
  for (long unsigned int i = 0; i < labelsGeneral.size(); i++)
  {
    auto instanceUuid = gen(instancesGeneral.at(i));
    data.labelsWithInstances.push_back(
        seerep_core_msgs::LabelWithInstance{ .label = labelsGeneral.at(i), .uuidInstance = instanceUuid });
  }

  std::vector<std::string> labelsBB;
  std::vector<std::vector<double>> boundingBoxes;
  std::vector<std::string> instances;
  readBoundingBoxLabeled(HDF5_GROUP_IMAGE, uuid, labelsBB, boundingBoxes, instances, false);

  for (long unsigned int i = 0; i < labelsBB.size(); i++)
  {
    auto instanceUuid = gen(instances.at(i));
    data.labelsWithInstances.push_back(
        seerep_core_msgs::LabelWithInstance{ .label = labelsBB.at(i), .uuidInstance = instanceUuid });
  }

  return data;
}

std::vector<std::string> Hdf5CoreImage::getDatasetUuids()
{
  return getGroupDatasets(HDF5_GROUP_IMAGE);
}

}  // namespace seerep_hdf5_core
