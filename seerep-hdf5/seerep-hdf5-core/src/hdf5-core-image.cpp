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

  data.header.frameId = getAttribute<std::string>(hdf5DatasetRawDataPath, *data_set_ptr,
                                                  seerep_hdf5_core::Hdf5CoreImage::HEADER_FRAME_ID);
  data.header.timestamp.seconds = getAttribute<int64_t>(hdf5DatasetRawDataPath, *data_set_ptr,
                                                        seerep_hdf5_core::Hdf5CoreImage::HEADER_STAMP_SECONDS);
  data.header.timestamp.nanos =
      getAttribute<int64_t>(hdf5DatasetRawDataPath, *data_set_ptr, seerep_hdf5_core::Hdf5CoreImage::HEADER_STAMP_NANOS);

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

  std::vector<std::string> labelsGeneral = readLabelsGeneral(hdf5DatasetPath);
  std::vector<std::string> labelsBB = readBoundingBoxLabels(hdf5DatasetPath);

  for (auto label : labelsGeneral)
  {
    data.labelsWithInstances.push_back(
        seerep_core_msgs::LabelWithInstance{ .label = label, .uuidInstance = boost::uuids::nil_uuid() });
  }
  for (auto label : labelsBB)
  {
    data.labelsWithInstances.push_back(
        seerep_core_msgs::LabelWithInstance{ .label = label, .uuidInstance = boost::uuids::nil_uuid() });
  }

  return data;
}

std::vector<std::string> Hdf5CoreImage::getDatasetUuids()
{
  return getGroupDatasets(HDF5_GROUP_IMAGE);
}

std::vector<std::string> Hdf5CoreImage::readLabelsGeneral(const std::string& dataGroup)
{
  std::string labelGeneralPath = dataGroup + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL;

  if (!m_file->exist(labelGeneralPath))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "id " << labelGeneralPath << " does not exist in file " << m_file->getName();
    return std::vector<std::string>();
  }

  std::vector<std::string> labels;
  HighFive::DataSet datasetLabels = m_file->getDataSet(labelGeneralPath);
  datasetLabels.read(labels);

  return labels;
}

std::vector<std::string> Hdf5CoreImage::readBoundingBoxLabels(const std::string& dataGroup)
{
  std::string labelBBPath = dataGroup + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB;
  if (!m_file->exist(labelBBPath))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "id " << labelBBPath << " does not exist in file " << m_file->getName();
    return std::vector<std::string>();
  }

  std::vector<std::string> labels;

  HighFive::DataSet datasetLabels = m_file->getDataSet(labelBBPath);
  datasetLabels.read(labels);

  return labels;
}

}  // namespace seerep_hdf5_core
