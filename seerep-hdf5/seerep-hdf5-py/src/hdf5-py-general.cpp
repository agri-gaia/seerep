#include "seerep-hdf5-py/hdf5-py-general.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_py
{

void Hdf5FileWrapper::createProject(const std::string& project_name, const std::string& root_frame_id)
{
  seerep_hdf5_core::Hdf5CoreGeneral io(getFile(), getMutex());

  io.writeProjectname(project_name);
  io.writeProjectFrameId(root_frame_id);
}

void Hdf5FileWrapper::setProjectGeolocation(const std::string& coordinate_system, const std::string& ellipsoid,
                                            double latitude, double longitude)
{
  // TODO
}

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

      HighFive::DataSpace labels_data_space = HighFive::DataSpace::From(labels);
      auto dataset_labels = getHdf5DataSet<std::string>(
          data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL + "_" + category_labels.category,
          labels_data_space);
      dataset_labels->write(labels);

      HighFive::DataSpace instances_data_space = HighFive::DataSpace::From(instance_uuids);
      auto dataset_instances =
          getHdf5DataSet<std::string>(data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES +
                                          "_" + category_labels.category,
                                      instances_data_space);
      dataset_instances->write(instance_uuids);
    }
  }
  m_file->flush();
}

std::vector<GeneralLabel> Hdf5PyGeneral::readLabelsGeneral(const std::string& data_group_id)
{
  auto hdf_group_ptr = getHdf5Group(data_group_id);

  std::vector<GeneralLabel> general_labels;

  const std::vector<std::string> group_datasets = hdf_group_ptr->listObjectNames();
  for (const auto& dataset_name : group_datasets)
  {
    if (dataset_name.rfind(seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES, 0) == 0)
    {
      std::string category = dataset_name.substr(seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES.length() + 1);
      general_labels.push_back(GeneralLabel(category));

      auto dataset_labels =
          getHdf5DataSet(data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL + "_" + category);
      auto dataset_instances = getHdf5DataSet(
          data_group_id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES + "_" + category);

      if (dataset_instances == nullptr || dataset_instances == nullptr)
      {
        throw std::invalid_argument("cannot find all general label datasets for category " + category);
      }

      std::vector<std::string> labels;
      std::vector<std::string> instance_uuids;

      dataset_labels->read(labels);
      dataset_instances->read(instance_uuids);

      if (labels.size() != instance_uuids.size())
      {
        throw std::invalid_argument("amounts for labels and instance uuids do not match for category " + category);
      }

      for (std::size_t i = 0; i < labels.size(); i++)
      {
        InstanceLabel label(labels[i], instance_uuids[i]);
        general_labels[general_labels.size() - 1].addLabel(label);
      }
    }
  }

  return general_labels;
}

} /* namespace seerep_hdf5_py */
