#ifndef SEEREP_CORE_MSGS_QUERY_RESULT_H_
#define SEEREP_CORE_MSGS_QUERY_RESULT_H_

#include <functional>

#include "datatype.h"
#include "query-result-project.h"

namespace seerep_core_msgs
{
struct QueryResult
{
  Datatype datatype;
  std::vector<QueryResultProject> queryResultProjects;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_QUERY_RESULT_H_
