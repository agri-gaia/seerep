#ifndef SEEREP_CORE_MSGS_QUERY_TF_H_
#define SEEREP_CORE_MSGS_QUERY_TF_H_

#include <functional>
#include <boost/uuid/uuid.hpp>  // uuid class

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
