#ifndef SEEREP_CORE_MSGS_LABEL_H_
#define SEEREP_CORE_MSGS_LABEL_H_

#include <string.h>

#include <boost/uuid/uuid.hpp>

namespace seerep_core_msgs
{
struct Label
{
  std::string label;
  int labelIdDatumaro;
  boost::uuids::uuid uuidInstance;
  int instanceIdDatumaro;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_LABEL_H_
