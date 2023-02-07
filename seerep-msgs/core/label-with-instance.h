#ifndef SEEREP_CORE_MSGS_LABEL_WITH_INSTANCE_H_
#define SEEREP_CORE_MSGS_LABEL_WITH_INSTANCE_H_

#include <boost/uuid/uuid.hpp>
#include <functional>

namespace seerep_core_msgs
{
struct LabelWithInstance
{
  std::string label;
  float labelConfidence;
  boost::uuids::uuid uuidInstance;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_LABEL_WITH_INSTANCE_H_
