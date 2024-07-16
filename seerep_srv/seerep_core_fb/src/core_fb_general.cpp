#include "seerep_core_fb/core_fb_pointcloud.h"

namespace seerep_core_fb
{
std::unordered_map<std::string, seerep_core_msgs::LabelDatumaro>
CoreFbGeneral::extractLabelsPerCategory(
    const seerep::fb::DatasetUuidLabel& datasetUuidLabel)
{
  boost::uuids::string_generator string_generator;
  std::unordered_map<std::string, seerep_core_msgs::LabelDatumaro>
      labelPerCategory;
  for (auto labelCategory : *datasetUuidLabel.labels())
  {
    if (labelCategory->labels())
    {
      std::vector<seerep_core_msgs::Label> labelVector;
      for (auto labelFb : *labelCategory->labels())
      {
        seerep_core_msgs::Label label;
        label.label = labelFb->label()->str();
        label.labelIdDatumaro = labelFb->labelIdDatumaro();
        label.instanceIdDatumaro = labelFb->instanceIdDatumaro();

        try
        {
          label.uuidInstance = string_generator(labelFb->instanceUuid()->str());
        }
        catch (std::runtime_error const& e)
        {
          label.uuidInstance = boost::uuids::nil_uuid();
        }
        labelVector.push_back(label);
      }
      seerep_core_msgs::LabelDatumaro labelDatumaro;
      labelDatumaro.datumaroJson = labelCategory->datumaroJson()->str();
      labelDatumaro.labels = labelVector;
      labelPerCategory.emplace(labelCategory->category()->str(), labelDatumaro);
    }
  }
  return labelPerCategory;
}
}  // namespace seerep_core_fb
