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
  // spatial
  typedef float PointDatatype;
  typedef bgm::point<PointDatatype, 3, bg::cs::cartesian> Point;
  typedef bgm::box<Point> AABB;
  typedef std::pair<AABB, uint64_t> AabbIdPair;
  typedef bgi::rtree<AabbIdPair, bgi::rstar<32>> rtree;

  // temporal
  typedef int64_t TimeDatatype;
  typedef bgm::point<TimeDatatype, 1, bg::cs::cartesian> TimePoint;
  typedef bgm::box<TimePoint> AabbTime;
  typedef std::pair<AabbTime, uint64_t> AabbTimeIdPair;
  typedef bgi::rtree<AabbTimeIdPair, bgi::rstar<32>> timetree;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_AABBHIERARCHY_H_
