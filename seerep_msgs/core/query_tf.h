#ifndef SEEREP_CORE_MSGS_QUERY_TF_H_
#define SEEREP_CORE_MSGS_QUERY_TF_H_

#include <boost/uuid/uuid.hpp>
#include <functional>

#include "timestamp.h"

namespace seerep_core_msgs
{
struct QueryTf
{
  boost::uuids::uuid project;
  Timestamp timestamp;
  std::string parentFrameId;
  std::string childFrameId;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_QUERY_TF_H_
