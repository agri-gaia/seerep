#ifndef SEEREP_CORE_MSGS_LABELS_WITH_INSTANCE_WITH_CATEGORY_H_
#define SEEREP_CORE_MSGS_LABELS_WITH_INSTANCE_WITH_CATEGORY_H_

#include <functional>

#include "label-with-instance.h"

namespace seerep_core_msgs
{
struct LabelsWithInstanceWithCategory
{
  std::string category;
  std::vector<std::string> labels;
  std::vector<std::string> instances;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_LABELS_WITH_INSTANCE_WITH_CATEGORY_H_
