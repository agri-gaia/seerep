#ifndef SEEREP_CORE_MSGS_DATASET_INDEXABLE_H_
#define SEEREP_CORE_MSGS_DATASET_INDEXABLE_H_

#include <functional>
#include <boost/uuid/uuid.hpp>  // uuid class

#include "aabb.h"

namespace seerep_core_msgs
{
struct DatasetIndexable
{
  TimePoint timestamp;
  AABB boundingbox;
  std::vector<std::string> labels;
  boost::uuids::uuid projectUuid;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_DATASET_INDEXABLE_H_
