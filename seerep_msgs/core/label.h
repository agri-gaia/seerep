#ifndef SEEREP_CORE_MSGS_LABEL_H_
#define SEEREP_CORE_MSGS_LABEL_H_

#include <string.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace seerep_core_msgs
{
struct Label
{
  std::string label = "";
  int labelIdDatumaro = -1;
  boost::uuids::uuid uuidInstance = boost::uuids::nil_uuid();
  int instanceIdDatumaro = -1;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_LABEL_H_
