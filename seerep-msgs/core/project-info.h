#ifndef SEEREP_CORE_MSGS_PROJECT_INFO_H_
#define SEEREP_CORE_MSGS_PROJECT_INFO_H_

#include <functional>
#include <boost/uuid/uuid.hpp>  // uuid class

namespace seerep_core_msgs
{
struct ProjectInfo
{
  std::string name;
  boost::uuids::uuid uuid;
  std::string frameId;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_PROJECT_INFO_H_
