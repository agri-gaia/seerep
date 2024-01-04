#ifndef SEEREP_CORE_MSGS_LABEL_CATEGORY_H_
#define SEEREP_CORE_MSGS_LABEL_CATEGORY_H_

#include <functional>

namespace seerep_core_msgs
{
struct LabelCategory
{
  std::string category;
  std::vector<std::string> labels;
  std::vector<int> labelsIdDatumaro;
  std::vector<std::string> instances;
  std::vector<int> instancesIdDatumaro;
  std::string datumaroJson;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_LABEL_CATEGORY_H_
