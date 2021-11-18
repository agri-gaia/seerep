#include "seerep-core/image.h"

namespace seerep_core
{
Image::Image(std::string coordinatesystemParent, std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io,
             const seerep::Image& image, const uint64_t& id, const boost::uuids::uuid& uuid)
  : m_coordinatesystemParent(coordinatesystemParent), m_hdf5_io(hdf5_io), m_id(id), m_uuid(uuid)
{
  m_hdf5_io->writeImage(boost::lexical_cast<std::string>(m_uuid), image);

  m_aabb = calcAABB(image);
  m_hdf5_io->writeAABB("images/" + boost::lexical_cast<std::string>(m_uuid), m_aabb);
}

Image::Image(std::string coordinatesystemParent, std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io, const uint64_t& id,
             const boost::uuids::uuid& uuid)
  : m_coordinatesystemParent(coordinatesystemParent), m_hdf5_io(hdf5_io), m_id(id), m_uuid(uuid)
{
  if (m_hdf5_io->hasAABB("images/" + boost::lexical_cast<std::string>(m_uuid)))
  {
    m_hdf5_io->readAABB("images/" + boost::lexical_cast<std::string>(m_uuid), m_aabb);
  }
  else
  {
  }
}

// Image::Image(std::string coordinatesystemParent, std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io,
//              const seerep::PointCloud2Labeled& pointcloud2labeled, const uint64_t& id)
//   : m_coordinatesystemParent(coordinatesystemParent), m_hdf5_io(hdf5_io), m_id(id)
// {
//   m_hdf5_io->writePointCloud2Labeled("pointclouds/" + std::to_string(m_id), pointcloud2labeled);
// }

Image::~Image()
{
}

std::optional<seerep::Image> Image::getData(const seerep::Boundingbox bb)
{
  std::cout << "loading image from images/" << m_id << std::endl;

  std::optional<seerep::Image> image = m_hdf5_io->readImage(boost::lexical_cast<std::string>(m_uuid));

  return image;
}

AabbHierarchy::AABB Image::getAABB()
{
  return m_aabb;
}

uint64_t Image::getID()
{
  return m_id;
}

boost::uuids::uuid Image::getUUID()
{
  return m_uuid;
}

AabbHierarchy::AABB Image::calcAABB(const seerep::Image& image)
{
  return AabbHierarchy::AABB(AabbHierarchy::Point(0, 0, 0), AabbHierarchy::Point(0, 0, 0));
}
} /* namespace seerep_core */
