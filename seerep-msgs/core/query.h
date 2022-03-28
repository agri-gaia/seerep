#ifndef SEEREP_CORE_MSGS_QUERY_H_
#define SEEREP_CORE_MSGS_QUERY_H_

#include <boost/uuid/uuid.hpp>  // uuid class
#include <functional>

#include "aabb.h"
#include "header.h"
#include "timeinterval.h"

namespace seerep_core_msgs
{
struct Query
{
  Header header;
  std::vector<boost::uuids::uuid> projects;
  AABB boundingbox;
  Timeinterval timeinterval;
  std::vector<std::string> label;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_QUERY_H_
