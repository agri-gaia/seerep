#ifndef SEEREP_CORE_IMAGE_H_
#define SEEREP_CORE_IMAGE_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/query.pb.h>
#include <seerep-msgs/image.pb.h>
// seerep-hdf5
#include <seerep-hdf5/io.h>
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
  Image(std::string coordinatesystemParent, std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io,
        const seerep::Image& image, const uint64_t& id, const boost::uuids::uuid& uuid);
  Image(std::string coordinatesystemParent, std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io, const uint64_t& id,
        const boost::uuids::uuid& uuid);
  ~Image();

  std::optional<seerep::Image> getData(const seerep::Query& query);

  AabbHierarchy::AABB getAABB();
  uint64_t getID();
  boost::uuids::uuid getUUID();
  int64_t getTime();
  AabbHierarchy::AabbTime getAABBTime();
  std::unordered_set<std::string> getLabels();

private:
  AabbHierarchy::AABB calcAABB();
  void recreateAABB();
  void recreateTime();
  void recreateLabel();

  std::string m_coordinatesystem;
  std::string m_coordinatesystemParent;
  std::shared_ptr<seerep_hdf5::SeerepHDF5IO> m_hdf5_io;
  const uint64_t m_id;
  const boost::uuids::uuid m_uuid;
  // axis aligned bounding box
  AabbHierarchy::AABB m_aabb;
  int64_t m_time;
  std::multimap<std::string, AabbHierarchy::AABB2D> m_labelsBB;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_IMAGE_H_
