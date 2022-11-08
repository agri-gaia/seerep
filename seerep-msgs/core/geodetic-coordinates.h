#ifndef SEEREP_CORE_MSGS_GEODETIC_COORDS_H_
#define SEEREP_CORE_MSGS_GEODETIC_COORDS_H_

#include <functional>

namespace seerep_core_msgs
{
struct geodeticCoordinates
{
  std::string coordinateSystem;
  std::string ellipsoid;

  int longitude;
  int latitude;
  int altitude;
};
}  // namespace seerep_core_msgs

#endif  // SEEREP_CORE_MSGS_GEODETIC_COORDS_H_
