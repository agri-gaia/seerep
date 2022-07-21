#ifndef SEEREP_CORE_MSGS_AABB_H_
#define SEEREP_CORE_MSGS_AABB_H_

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

// uuid
#include <boost/uuid/uuid.hpp>  // uuid class

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
namespace bgm = boost::geometry::model;

namespace seerep_core_msgs
{
// spatial 3D
typedef float PointDatatype;
typedef bgm::point<PointDatatype, 3, bg::cs::cartesian> Point;
typedef bgm::box<Point> AABB;  // axis-aligned bounding box
typedef std::pair<AABB, boost::uuids::uuid> AabbIdPair;
typedef bgi::rtree<AabbIdPair, bgi::rstar<32>> rtree;

// spatial 2D
typedef float Point2DDatatype;
typedef bgm::point<Point2DDatatype, 2, bg::cs::cartesian> Point2D;
typedef bgm::box<Point2D> AABB2D;  // axis-aligned bounding box
typedef std::pair<AABB2D, boost::uuids::uuid> Aabb2DIdPair;
typedef bgi::rtree<Aabb2DIdPair, bgi::rstar<32>> rtree2D;

// temporal
typedef int64_t TimeDatatype;
typedef bgm::point<TimeDatatype, 1, bg::cs::cartesian> TimePoint;
typedef bgm::box<TimePoint> AabbTime;  // axis-aligned bounding box
typedef std::pair<AabbTime, boost::uuids::uuid> AabbTimeIdPair;
typedef bgi::rtree<AabbTimeIdPair, bgi::rstar<32>> timetree;

}  // namespace seerep_core_msgs

#endif  // SEEREP_CORE_MSGS_AABB_H_
