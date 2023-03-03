#ifndef SEEREP_CORE_MSGS_CAMERA_INTRINSICS_QUERY_H_
#define SEEREP_CORE_MSGS_CAMERA_INTRINSICS_QUERY_H_

#include <boost/uuid/uuid.hpp>

namespace seerep_core_msgs
{
struct camera_intrinsics_query
{
  boost::uuids::uuid uuidProject;
  boost::uuids::uuid uuidCameraIntrinsics;
};
}  // namespace seerep_core_msgs

#endif  // SEEREP_CORE_MSGS_CAMERA_INTRINSICS_QUERY_H_
