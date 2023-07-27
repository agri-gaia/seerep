#ifndef SEEREP_CORE_MSGS_QUERY_H_
#define SEEREP_CORE_MSGS_QUERY_H_

#include <boost/uuid/uuid.hpp>
#include <functional>

#include "header.h"
#include "polygon2d.h"
#include "timeinterval.h"

namespace seerep_core_msgs
{
struct Query
{
  Header header;
  std::optional<std::vector<boost::uuids::uuid>> projects;  ///< search all projects if not set
  std::optional<Polygon2D> polygon;                         // query dataset in the region defined by this polygon
  bool fullyEncapsulated;                    // if true, only return results fully inside the polygon defined above
  std::optional<Timeinterval> timeinterval;  ///< only do temporal query if set
  std::optional<std::unordered_map<std::string, std::vector<std::string>>> label;  ///< only do semantic query if set
  bool mustHaveAllLabels;  ///< a dataset only fulfills semantic query if all labels are present
  std::optional<std::vector<boost::uuids::uuid>> instances;  ///< only query instances if set
  std::optional<std::vector<boost::uuids::uuid>> dataUuids;  ///< only filter by data uuid if set
  bool withoutData;                                          ///< do not return the data itself if set
  uint maxNumData;                                           ///< max number of datasets that should be returned
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_QUERY_H_
