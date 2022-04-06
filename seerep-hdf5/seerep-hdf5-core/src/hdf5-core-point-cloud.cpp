#include "seerep-hdf5-core/hdf5-core-point-cloud.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_core
{
Hdf5CorePointCloud::Hdf5CorePointCloud(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx)
{
}

std::optional<seerep_core_msgs::DatasetIndexable> Hdf5CorePointCloud::readPointCloud(const boost::uuids::uuid& uuid)
{
  return readPointCloud(boost::lexical_cast<std::string>(uuid));
}

std::optional<seerep_core_msgs::DatasetIndexable> Hdf5CorePointCloud::readPointCloud(const std::string& uuid)
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

  std::vector<std::string> labelsGeneral = readLabelsGeneral(hdf5DatasetPath);
  std::vector<std::string> labelsBB = readBoundingBoxLabels(hdf5DatasetPath);

  data.labels.insert(std::end(data.labels), std::begin(labelsGeneral), std::end(labelsGeneral));
  data.labels.insert(std::end(data.labels), std::begin(labelsBB), std::end(labelsBB));

  return data;
}

std::vector<std::string> Hdf5CorePointCloud::readLabelsGeneral(const std::string& dataGroup)
{
  if (!m_file->exist(dataGroup + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL))
  {
    std::cout << "id " << dataGroup + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL
              << " does not exist in file " << m_file->getName() << std::endl;
    return std::vector<std::string>();
  }

  std::vector<std::string> labels;
  HighFive::DataSet datasetLabels =
      m_file->getDataSet(dataGroup + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL);
  datasetLabels.read(labels);

  return labels;
}

std::vector<std::string> Hdf5CorePointCloud::readBoundingBoxLabels(const std::string& dataGroup)
{
  if (!m_file->exist(dataGroup + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB))
  {
    std::cout << "id " << dataGroup + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB << " does not exist in file "
              << m_file->getName() << std::endl;
    return std::vector<std::string>();
  }

  std::vector<std::string> labels;

  HighFive::DataSet datasetLabels = m_file->getDataSet(dataGroup + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB);
  datasetLabels.read(labels);

  return labels;
}

}  // namespace seerep_hdf5_core
