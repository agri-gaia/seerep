#ifndef SEEREP_CORE_MSGS_SPARQL_QUERY_H_
#define SEEREP_CORE_MSGS_SPARQL_QUERY_H_

namespace seerep_core_msgs
{
struct SparqlQuery
{
  std::string category;
  std::string sparql;
  std::string variableNameOfCategory;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_SPARQL_QUERY_H_
