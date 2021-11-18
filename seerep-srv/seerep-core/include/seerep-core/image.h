#ifndef SEEREP_CORE_IMAGE_H_
#define SEEREP_CORE_IMAGE_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/boundingbox.pb.h>
#include <seerep-msgs/image.pb.h>
// #include <seerep-msgs/point_cloud_2_labeled.pb.h>
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
  // labeled
  // Pointcloud(std::string coordinatesystemParent, std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io,
  //            const seerep::ImageLabeled& image, const uint64_t& id);
  ~Image();

  std::optional<seerep::Image> getData(const seerep::Boundingbox bb);

  AabbHierarchy::AABB getAABB();
  uint64_t getID();
  boost::uuids::uuid getUUID();

private:
  AabbHierarchy::AABB calcAABB(const seerep::Image& image);

  std::string m_coordinatesystem;
  std::string m_coordinatesystemParent;
  std::shared_ptr<seerep_hdf5::SeerepHDF5IO> m_hdf5_io;
  const uint64_t m_id;
  const boost::uuids::uuid m_uuid;
  // axis aligned bounding box
  AabbHierarchy::AABB m_aabb;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_IMAGE_H_
