#ifndef SEEREP_CORE_MSGS_H_
#define SEEREP_CORE_MSGS_H_

#include <functional>

namespace seerep_core_msgs
{
struct Timestamp
{
  int64_t seconds;
  int64_t nanos;
};
} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_H_
