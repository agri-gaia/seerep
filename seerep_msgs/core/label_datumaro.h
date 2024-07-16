#ifndef SEEREP_CORE_MSGS_LABEL_DATUMARO_H_
#define SEEREP_CORE_MSGS_LABEL_DATUMARO_H_

#include <string.h>

#include <boost/uuid/uuid.hpp>

#include "label.h"

namespace seerep_core_msgs
{
struct LabelDatumaro
{
  std::vector<Label> labels;
  std::string datumaroJson;
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_LABEL_DATUMARO_H_
