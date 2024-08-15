#ifndef SEEREP_CORE_MSGS_POLYGON2D_H_
#define SEEREP_CORE_MSGS_POLYGON2D_H_

#include "aabb.h"

namespace seerep_core_msgs
{
struct Polygon2D
{
  std::vector<Point2D> vertices;
  double z;
  double height;
};
}  // namespace seerep_core_msgs

#endif
