#ifndef SEEREP_CORE_MSGS_HEADER_H_
#define SEEREP_CORE_MSGS_HEADER_H_

#include <boost/uuid/uuid.hpp>

#include "datatype.h"
#include "timestamp.h"

namespace seerep_core_msgs
{
struct Header
{
  Datatype datatype;
  std::string frameId;
  Timestamp timestamp;
  boost::uuids::uuid uuidProject;
  boost::uuids::uuid uuidData;
  uint32_t sequence;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_HEADER_H_
