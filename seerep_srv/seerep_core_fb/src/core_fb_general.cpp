#include "seerep_core_fb/core_fb_pointcloud.h"

namespace seerep_core_fb
{
std::unordered_map<std::string, std::vector<seerep_core_msgs::Label>>
CoreFbGeneral::extractLabelsPerCategory(const seerep::fb::DatasetUuidLabel& datasetUuidLabel)
{
  boost::uuids::string_generator string_generator;
  std::unordered_map<std::string, std::vector<seerep_core_msgs::Label>> labelPerCategory;
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
      labelPerCategory.emplace(labelCategory->category()->str(), labelVector);
    }
  }
  return labelPerCategory;
}
}  // namespace seerep_core_fb
