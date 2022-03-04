#ifndef SEEREP_CORE_MSGS_HEADER_H_
#define SEEREP_CORE_MSGS_HEADER_H_

#include "timestamp.h"
#include <boost/uuid/uuid.hpp>

namespace seerep_core_msgs
{
struct Header
{
  std::string frameId;
  Timestamp timestamp;
  boost::uuids::uuid uuidProject;
  boost::uuids::uuid uuidData;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_HEADER_H_
