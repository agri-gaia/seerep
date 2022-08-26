#include "seerep-hdf5-core/hdf5-core-point-cloud.h"

#include <highfive/H5DataSet.hpp>

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
  std::string hdf5DatasetRawDataPath = hdf5DatasetPath + "/" + seerep_hdf5_core::Hdf5CorePointCloud::RAWDATA;

  if (!m_file->exist(hdf5DatasetPath) || !m_file->exist(hdf5DatasetRawDataPath))
    return std::nullopt;

  std::shared_ptr<HighFive::Group> group_ptr = std::make_shared<HighFive::Group>(m_file->getGroup(hdf5DatasetPath));
  std::shared_ptr<HighFive::DataSet> data_set_ptr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRawDataPath));

  seerep_core_msgs::DatasetIndexable data;

  boost::uuids::string_generator gen;
  data.header.uuidData = gen(uuid);

  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::HEADER_FRAME_ID).read(data.header.frameId);
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::HEADER_STAMP_SECONDS).read(data.header.timestamp.seconds);
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::HEADER_STAMP_NANOS).read(data.header.timestamp.nanos);

  std::vector<float> bb;
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::BOUNDINGBOX).write(bb);
  data.boundingbox.min_corner().set<0>(bb.at(0));
  data.boundingbox.min_corner().set<1>(bb.at(1));
  data.boundingbox.min_corner().set<2>(bb.at(2));
  data.boundingbox.max_corner().set<0>(bb.at(3));
  data.boundingbox.max_corner().set<1>(bb.at(4));
  data.boundingbox.max_corner().set<2>(bb.at(5));

  std::vector<std::string> labelsGeneral;
  std::vector<std::string> instancesGeneral;
  readLabelsGeneral(HDF5_GROUP_POINTCLOUD, uuid, labelsGeneral, instancesGeneral);
  for (long unsigned int i = 0; i < labelsGeneral.size(); i++)
  {
    auto instanceUuid = gen(instancesGeneral.at(i));
    data.labelsWithInstances.push_back(
        seerep_core_msgs::LabelWithInstance{ .label = labelsGeneral.at(i), .uuidInstance = instanceUuid });
  }

  std::vector<std::string> labelsBB;
  std::vector<std::vector<double>> boundingBoxes;
  std::vector<std::string> instances;
  readBoundingBoxLabeled(HDF5_GROUP_POINTCLOUD, uuid, labelsBB, boundingBoxes, instances, false);

  for (long unsigned int i = 0; i < labelsBB.size(); i++)
  {
    auto instanceUuid = gen(instances.at(i));
    data.labelsWithInstances.push_back(
        seerep_core_msgs::LabelWithInstance{ .label = labelsBB.at(i), .uuidInstance = instanceUuid });
  }

  return data;
}

std::vector<std::string> Hdf5CorePointCloud::getDatasetUuids()
{
  return getGroupDatasets(HDF5_GROUP_POINTCLOUD);
}

}  // namespace seerep_hdf5_core
