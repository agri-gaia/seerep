#ifndef SEEREP_CORE_MSGS_GEODETIC_COORDS_H_
#define SEEREP_CORE_MSGS_GEODETIC_COORDS_H_

#include <functional>

namespace seerep_core_msgs
{
struct GeodeticCoordinates
{
  std::string coordinateSystem;

  double altitude;
  double latitude;
  double longitude;
};
}  // namespace seerep_core_msgs

#endif  // SEEREP_CORE_MSGS_GEODETIC_COORDS_H_
