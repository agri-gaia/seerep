#include "seerep_hdf5_pb/hdf5_pb_general.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_pb
{
Hdf5PbGeneral::Hdf5PbGeneral(std::shared_ptr<HighFive::File>& file,
                             std::shared_ptr<std::mutex>& write_mtx)
  : seerep_hdf5_core::Hdf5CoreGeneral(file, write_mtx)
{
}

void Hdf5PbGeneral::writeLabels(
    const std::string& datatypeGroup, const std::string& uuid,
    const google::protobuf::RepeatedPtrField<seerep::pb::LabelCategory>&
        labelsCategoryPb)
{
  if (!labelsCategoryPb.empty())
  {
    std::vector<seerep_core_msgs::LabelCategory> labelCategoryVector;
    for (auto& labelCategoryPb : labelsCategoryPb)
    {
      std::vector<std::string> labels, instances;
      std::vector<int> labelsIdDatumaro, instancesIdDatumaro;
      for (auto& label : labelCategoryPb.labels())
      {
        labels.push_back(label.label());
        labelsIdDatumaro.push_back(label.labeliddatumaro());
        instances.push_back(label.instanceuuid());
        instancesIdDatumaro.push_back(label.instanceiddatumaro());
      }
      seerep_core_msgs::LabelCategory labelCategory;
      labelCategory.category = labelCategoryPb.category();
      labelCategory.labels = labels;
      labelCategory.labelsIdDatumaro = labelsIdDatumaro;
      labelCategory.instances = instances;
      labelCategory.instancesIdDatumaro = instancesIdDatumaro;
      labelCategory.datumaroJson = labelCategoryPb.datumarojson();
      labelCategoryVector.push_back(labelCategory);
    }
    Hdf5CoreGeneral::writeLabels(datatypeGroup, uuid, labelCategoryVector);
  }
}

std::optional<google::protobuf::RepeatedPtrField<seerep::pb::LabelCategory>>
Hdf5PbGeneral::readLabels(const std::string& datatypeGroup,
                          const std::string& uuid)
{
  std::vector<std::string> labelCategories, datumaroJsonPerCategory;
  std::vector<std::vector<seerep_core_msgs::Label>> labelsPerCategory;
  seerep_hdf5_core::Hdf5CoreGeneral::readLabels(datatypeGroup, uuid,
                                                labelCategories,
                                                labelsPerCategory,
                                                datumaroJsonPerCategory);

  google::protobuf::RepeatedPtrField<seerep::pb::LabelCategory> result;

  for (size_t iCategory = 0; iCategory < labelCategories.size(); iCategory++)
  {
    seerep::pb::LabelCategory resultCat;
    resultCat.set_category(labelCategories.at(iCategory));
    resultCat.set_datumarojson(datumaroJsonPerCategory.at(iCategory));
    for (size_t i = 0; i < labelsPerCategory.at(iCategory).size(); i++)
    {
      auto label = resultCat.add_labels();
      label->set_label(labelsPerCategory.at(iCategory).at(i).label);
      label->set_labeliddatumaro(
          labelsPerCategory.at(iCategory).at(i).labelIdDatumaro);
      label->set_instanceuuid(boost::lexical_cast<std::string>(
          labelsPerCategory.at(iCategory).at(i).uuidInstance));
      label->set_instanceiddatumaro(
          labelsPerCategory.at(iCategory).at(i).instanceIdDatumaro);
    }
    result.Add(std::move(resultCat));
  }
  return result;
}

} /* namespace seerep_hdf5_pb */
