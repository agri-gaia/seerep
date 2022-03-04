#ifndef SEEREP_CORE_MSGS_QUERY_H_
#define SEEREP_CORE_MSGS_QUERY_H_

#include <functional>
#include <boost/uuid/uuid.hpp>  // uuid class
#include "aabb.h"
#include "timeinterval.h"
#include "header.h"

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
