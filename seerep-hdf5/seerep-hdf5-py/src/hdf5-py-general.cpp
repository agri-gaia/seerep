#include "seerep-hdf5-py/hdf5-py-general.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_py
{

Hdf5PyGeneral::Hdf5PyGeneral(Hdf5FileWrapper& hdf5_file) : Hdf5CoreGeneral(hdf5_file.getFile(), hdf5_file.getMutex())
{
}

void Hdf5PyGeneral::writeLabelsGeneral(const std::string& data_group_id,
                                       const std::vector<seerep_hdf5_py::GeneralLabel>& general_labels)
{
  for (const auto& category_labels : general_labels)
  {
    if (!category_labels.labels.empty())
    {
      std::vector<std::string> labels;
      std::vector<std::string> instance_uuids;

      labels.reserve(category_labels.labels.size());
      instance_uuids.reserve(category_labels.labels.size());

      for (const auto& instance_label : category_labels.labels)
      {
        labels.push_back(instance_label.label);
        instance_uuids.push_back(instance_label.instance_uuid);
      }

      HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
          data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL + "_" + category_labels.category,
          HighFive::DataSpace::From(labels));
      datasetLabels.write(labels);

      HighFive::DataSet datasetInstances = m_file->createDataSet<std::string>(
          data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES + "_" +
              category_labels.category,
          HighFive::DataSpace::From(instance_uuids));
      datasetInstances.write(instance_uuids);
    }
  }
  m_file->flush();
}

std::shared_ptr<std::vector<seerep_hdf5_py::GeneralLabel>>
Hdf5PyGeneral::readLabelsGeneral(const std::string& data_group_id)
{
  auto hdf_group_ptr = getHdf5Group(data_group_id);

  std::shared_ptr<std::vector<seerep_hdf5_py::GeneralLabel>> general_labels;

  const std::vector<std::string> group_datasets = hdf_group_ptr->listObjectNames();
  for (const auto& dataset_name : group_datasets)
  {
    if (dataset_name.rfind(seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL, 0) == 0)
    {
      std::string category = dataset_name.substr(seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL.length());
      general_labels->push_back(seerep_hdf5_py::GeneralLabel(category));

      checkExists(data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL + "_" + category);
      checkExists(data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES + "_" + category);
      HighFive::DataSet dataset_labels =
          m_file->getDataSet(data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL + "_" + category);
      HighFive::DataSet dataset_instances = m_file->getDataSet(
          data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES + "_" + category);

      std::vector<std::string> labels;
      std::vector<std::string> instance_uuids;

      dataset_labels.read(labels);
      dataset_instances.read(instance_uuids);

      if (labels.size() != instance_uuids.size())
      {
        throw std::invalid_argument("amounts for labels and instance uuids do not match for catrgory " + category);
      }

      for (std::size_t i = 0; i < labels.size(); i++)
      {
        InstanceLabel label(labels[i], instance_uuids[i]);
        general_labels->at(general_labels->size() - 1).addLabel(label);
      }
    }
  }

  return general_labels;
}

} /* namespace seerep_hdf5_py */
