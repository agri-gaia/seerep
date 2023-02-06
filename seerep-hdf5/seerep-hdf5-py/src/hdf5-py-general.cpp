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
  for (auto& category_labels : general_labels)
  {
    if (!category_labels.labels.empty())
    {
      std::vector<std::string> labels;
      std::vector<std::string> instance_uuids;

      labels.reserve(category_labels.labels.size());
      instance_uuids.reserve(category_labels.labels.size());

      for (auto& instance_label : category_labels.labels)
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

} /* namespace seerep_hdf5_py */
