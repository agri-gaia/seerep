#ifndef SEEREP_CORE_IMAGE_H_
#define SEEREP_CORE_IMAGE_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/query.pb.h>
#include <seerep-msgs/image.pb.h>
// seerep-hdf5
#include <seerep-hdf5/ioImage.h>
// seerep-conversion
#include <seerep_ros_conversions/conversions.h>

// seerep-core
#include "aabb-hierarchy.h"

// uuid
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <boost/lexical_cast.hpp>
#include <boost/functional/hash.hpp>

namespace seerep_core
{
class Image
{
public:
  Image(std::shared_ptr<seerep_hdf5::SeerepHDF5IOImage> hdf5_io, const seerep::Image& image, const uint64_t& id,
        const boost::uuids::uuid& uuid);
  Image(std::shared_ptr<seerep_hdf5::SeerepHDF5IOImage> hdf5_io, const uint64_t& id, const boost::uuids::uuid& uuid);
  ~Image();

  std::optional<seerep::Image> getData(const seerep::Query& query);

  AabbHierarchy::AABB getAABB();
  uint64_t getID();
  boost::uuids::uuid getUUID();
  void getTime(int64_t& timeSecs, int64_t& timeNanos);
  AabbHierarchy::AabbTime getAABBTime();
  std::unordered_set<std::string> getLabels();
  std::string getFrameId();

private:
  AabbHierarchy::AABB calcAABB();
  void recreateAABB();
  void recreateTime();
  void recreateLabel();
  void storeLabelGeneral(google::protobuf::RepeatedPtrField<std::string> labelGeneral);
  void storeLabelBB(google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeled> labelsBB);

  std::string m_frameId;
  std::shared_ptr<seerep_hdf5::SeerepHDF5IOImage> m_hdf5_io;
  const uint64_t m_id;
  const boost::uuids::uuid m_uuid;
  // axis aligned bounding box
  AabbHierarchy::AABB m_aabb;
  int64_t m_timeSecs;
  int64_t m_timeNanos;
  std::multimap<std::string, AabbHierarchy::AABB2D> m_labelsBB;
  std::unordered_set<std::string> m_labelGeneral;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_IMAGE_H_
