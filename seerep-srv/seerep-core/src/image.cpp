#include "seerep-core/image.h"

namespace seerep_core
{
// constructor when data received and stored to hdf5
Image::Image(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io, const seerep::Image& image, const uint64_t& id,
             const boost::uuids::uuid& uuid)
  : m_hdf5_io(hdf5_io), m_id(id), m_uuid(uuid)
{
  m_hdf5_io->writeImage(boost::lexical_cast<std::string>(m_uuid), image);
  m_frameId = image.header().frame_id();

  // space
  m_aabb = calcAABB();
  m_hdf5_io->writeAABB(seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE, boost::lexical_cast<std::string>(m_uuid), m_aabb);

  // time
  m_timeSecs = image.header().stamp().seconds();

  // semantic
  if (!image.labels_general().empty())
  {
    storeLabelGeneral(image.labels_general());
  }

  if (!image.labels_bb().empty())
  {
    storeLabelBB(image.labels_bb());
  }
}

// constructor if recreating the server from hdf5
Image::Image(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io, const uint64_t& id, const boost::uuids::uuid& uuid)
  : m_hdf5_io(hdf5_io), m_id(id), m_uuid(uuid)
{
  auto frameId =
      m_hdf5_io->readFrameId(seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE, boost::lexical_cast<std::string>(m_uuid));
  m_frameId = frameId.value_or("noframeid");

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
  return m_timeSecs;
}

AabbHierarchy::AabbTime Image::getAABBTime()
{
  return AabbHierarchy::AabbTime(AabbHierarchy::TimePoint(m_timeSecs), AabbHierarchy::TimePoint(m_timeSecs));
}

std::unordered_set<std::string> Image::getLabels()
{
  std::unordered_set<std::string> labelset;

  // add all bounding box based labels to the result set
  for (auto label : m_labelsBB)
  {
    labelset.insert(label.first);
  }

  // add all general labels to the result set
  labelset.insert(m_labelGeneral.begin(), m_labelGeneral.end());

  return labelset;
}

std::string Image::getFrameId()
{
  return m_frameId;
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
    m_timeSecs =
        m_hdf5_io->readTime(seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE, boost::lexical_cast<std::string>(m_uuid));
  }
  else
  {
    if (m_hdf5_io->hasTimeRaw(seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE, boost::lexical_cast<std::string>(m_uuid)))
    {
      m_timeSecs = m_hdf5_io->readTimeFromRaw(seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE,
                                              boost::lexical_cast<std::string>(m_uuid));
      m_hdf5_io->writeTime(seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE, boost::lexical_cast<std::string>(m_uuid),
                           m_timeSecs);
    }
  }
}

void Image::recreateLabel()
{
  std::optional<google::protobuf::RepeatedPtrField<std::string>> labelGeneral = m_hdf5_io->readLabelsGeneral(
      seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE, boost::lexical_cast<std::string>(m_uuid));
  if (labelGeneral)
  {
    storeLabelGeneral(labelGeneral.value());
  }
  std::optional<google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeled>> labelsBB =
      m_hdf5_io->readBoundingBox2DLabeled(seerep_hdf5::SeerepHDF5IO::HDF5_GROUP_IMAGE,
                                          boost::lexical_cast<std::string>(m_uuid));

  if (labelsBB)
  {
    storeLabelBB(labelsBB.value());
  }
}

void Image::storeLabelGeneral(google::protobuf::RepeatedPtrField<std::string> labelGeneral)
{
  for (auto label : labelGeneral)
  {
    m_labelGeneral.insert(label);
  }
}

void Image::storeLabelBB(google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeled> labelsBB)
{
  for (auto label : labelsBB)
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

} /* namespace seerep_core */
