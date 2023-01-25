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

}  // namespace seerep_hdf5_core
