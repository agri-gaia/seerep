#ifndef SEEREP_CORE_MSGS_TIMEINTERVAL_H_
#define SEEREP_CORE_MSGS_TIMEINTERVAL_H_

#include "Timestamp.h"

namespace seerep_core_msgs
{
struct Timeinterval
{
  Timestamp timeMin;
  Timestamp timeMax;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_TIMEINTERVAL_H_
