#ifndef SEEREP_CORE_MSGS_DATASET_INDEXABLE_H_
#define SEEREP_CORE_MSGS_DATASET_INDEXABLE_H_

#include <functional>

#include "aabb.h"
#include "header.h"
#include "label.h"

namespace seerep_core_msgs
{
struct DatasetIndexable
{
  Header header;
  AABB boundingbox;
  std::unordered_map<std::string, std::vector<Label>> labelsCategory;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_DATASET_INDEXABLE_H_
