#ifndef SEEREP_CORE_MSGS_QUERY_RESULT_PROJECT_H_
#define SEEREP_CORE_MSGS_QUERY_RESULT_PROJECT_H_

#include <functional>
#include <boost/uuid/uuid.hpp>  // uuid class

namespace seerep_core_msgs
{
struct QueryResultProject
{
  boost::uuids::uuid projectUuid;
  std::vector<boost::uuids::uuid> dataUuids;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_QUERY_RESULT_PROJECT_H_
