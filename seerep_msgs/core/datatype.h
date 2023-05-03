#ifndef SEEREP_CORE_MSGS_DATATYPE_H_
#define SEEREP_CORE_MSGS_DATATYPE_H_

namespace seerep_core_msgs
{
/** @brief enum class for the data types */
enum class Datatype
{
  Unknown,  // default value
  Image,
  PointCloud,
  Point,
  All
};

} /* namespace seerep_core_msgs */

#endif  // SEEREP_CORE_MSGS_DATATYPE_H_
