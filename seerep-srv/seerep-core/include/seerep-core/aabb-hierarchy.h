#ifndef SEEREP_CORE_AABBHIERARCHY_H_
#define SEEREP_CORE_AABBHIERARCHY_H_

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
namespace bgm = boost::geometry::model;

namespace seerep_core
{
class AabbHierarchy
{
public:
  typedef bgm::point<float, 3, bg::cs::cartesian> Point;
  typedef bgm::box<Point> AABB;
  typedef std::pair<AABB, uint64_t> AabbIdPair;
  typedef bgi::rtree<AabbIdPair, bgi::rstar<32>> rtree;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_AABBHIERARCHY_H_
