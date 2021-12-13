#include "seerep-core/image.h"

namespace seerep_core
{
// constructor when data received and stored to hdf5
Image::Image(std::string coordinatesystemParent, std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io,
             const seerep::Image& image, const uint64_t& id, const boost::uuids::uuid& uuid)
  : m_coordinatesystemParent(coordinatesystemParent), m_hdf5_io(hdf5_io), m_id(id), m_uuid(uuid)
{
  m_hdf5_io->writeImage(boost::lexical_cast<std::string>(m_uuid), image);

  // space
  m_aabb = calcAABB();
  m_hdf5_io->writeAABB(seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE, boost::lexical_cast<std::string>(m_uuid), m_aabb);

  // time
  m_time = image.header().stamp().seconds();

  // semantic
  if (!image.labels_bb().empty())
  {
    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    for (auto label : image.labels_bb())
    {
      m_labelsBB.insert(std::make_pair(
          // key = label as string
          label.label(),
          // value = 2D bounding box
          AabbHierarchy::AABB2D(
              AabbHierarchy::Point2D(label.boundingbox().point_min().x(), label.boundingbox().point_min().y()),
              AabbHierarchy::Point2D(label.boundingbox().point_max().x(), label.boundingbox().point_max().y()))));
    }
  }
}

// constructor if recreating the server from hdf5
Image::Image(std::string coordinatesystemParent, std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io, const uint64_t& id,
             const boost::uuids::uuid& uuid)
  : m_coordinatesystemParent(coordinatesystemParent), m_hdf5_io(hdf5_io), m_id(id), m_uuid(uuid)
{
  recreateAABB();
  recreateTime();
  recreateLabel();
}

Image::~Image()
{
}

std::optional<seerep::Image> Image::getData(const seerep::Query& query)
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

int64_t Image::getTime()
{
  return m_time;
}

AabbHierarchy::AabbTime Image::getAABBTime()
{
  return AabbHierarchy::AabbTime(AabbHierarchy::TimePoint(m_time), AabbHierarchy::TimePoint(m_time));
}

std::unordered_set<std::string> Image::getLabels()
{
  std::unordered_set<std::string> labelset;

  for (auto label : m_labelsBB)
  {
    labelset.emplace(label.first);
  }

  return labelset;
}

AabbHierarchy::AABB Image::calcAABB()
{
  return AabbHierarchy::AABB(AabbHierarchy::Point(0, 0, 0), AabbHierarchy::Point(0, 0, 0));
}

void Image::recreateAABB()
{
  if (m_hdf5_io->hasAABB(seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE, boost::lexical_cast<std::string>(m_uuid)))
  {
    m_hdf5_io->readAABB(seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE, boost::lexical_cast<std::string>(m_uuid), m_aabb);
  }
  else
  {
    m_aabb = calcAABB();
    m_hdf5_io->writeAABB(seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE, boost::lexical_cast<std::string>(m_uuid), m_aabb);
  }
}

void Image::recreateTime()
{
  if (m_hdf5_io->hasTime(seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE, boost::lexical_cast<std::string>(m_uuid)))
  {
    m_time = m_hdf5_io->readTime(seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE, boost::lexical_cast<std::string>(m_uuid));
  }
  else
  {
    if (m_hdf5_io->hasTimeRaw(seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE, boost::lexical_cast<std::string>(m_uuid)))
    {
      m_time = m_hdf5_io->readTimeFromRaw(seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE,
                                          boost::lexical_cast<std::string>(m_uuid));
      m_hdf5_io->writeTime(seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE, boost::lexical_cast<std::string>(m_uuid),
                           m_time);
    }
  }
}

void Image::recreateLabel()
{
  std::optional<google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeled>> labelsBB =
      m_hdf5_io->readBoundingBox2DLabeled(seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE,
                                          boost::lexical_cast<std::string>(m_uuid));

  if (labelsBB)
  {
    for (auto labelBB : labelsBB.value())
    {
      AabbHierarchy::AABB2D aabb2d(
          AabbHierarchy::Point2D(labelBB.boundingbox().point_min().x(), labelBB.boundingbox().point_min().y()),
          AabbHierarchy::Point2D(labelBB.boundingbox().point_max().x(), labelBB.boundingbox().point_max().y()));
      m_labelsBB.insert(std::make_pair(labelBB.label(), aabb2d));
    }
  }
}
} /* namespace seerep_core */
