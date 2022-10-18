#include "seerep-hdf5-core/hdf5-core-general.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_core
{
Hdf5CoreGeneral::Hdf5CoreGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : m_file(file), m_write_mtx(write_mtx)
{
}

std::vector<std::string> Hdf5CoreGeneral::getGroupDatasets(const std::string& id)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::vector<std::string> rootObjects = m_file->listObjectNames();

  if (id.empty())
  {
    return rootObjects;
  }
  else
  {
    // check if rootObjects contains the group id
    if (std::find(rootObjects.begin(), rootObjects.end(), id) != rootObjects.end())
    {
      return m_file->getGroup(id).listObjectNames();
    }
    else
    {
      return std::vector<std::string>();
    }
  }
}

void Hdf5CoreGeneral::writeProjectname(const std::string& projectname)
{
  const std::scoped_lock lock(*m_write_mtx);

  if (!m_file->hasAttribute(PROJECTNAME))
  {
    m_file->createAttribute<std::string>(PROJECTNAME, projectname);
  }
  else
  {
    m_file->getAttribute(PROJECTNAME).write(projectname);
  }
  m_file->flush();
}

std::string Hdf5CoreGeneral::readProjectname()
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string projectname;
  if (m_file->hasAttribute(PROJECTNAME))
  {
    m_file->getAttribute(PROJECTNAME).read(projectname);
  }
  return projectname;
}

void Hdf5CoreGeneral::writeProjectFrameId(const std::string& frameId)
{
  const std::scoped_lock lock(*m_write_mtx);

  if (!m_file->hasAttribute(PROJECTFRAMEID))
  {
    m_file->createAttribute<std::string>(PROJECTFRAMEID, frameId);
  }
  else
  {
    m_file->getAttribute(PROJECTFRAMEID).write(frameId);
  }
  m_file->flush();
}

std::string Hdf5CoreGeneral::readProjectFrameId()
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string frameId;
  if (m_file->hasAttribute(PROJECTFRAMEID))
  {
    m_file->getAttribute(PROJECTFRAMEID).read(frameId);
  }
  return frameId;
}

std::optional<std::string> Hdf5CoreGeneral::readFrameId(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  std::string hdf5DatasetRawDataPath = id + "/" + RAWDATA;

  checkExists(hdf5DatasetRawDataPath);

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "get dataset " << hdf5DatasetRawDataPath;
  std::shared_ptr<HighFive::DataSet> data_set_ptr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRawDataPath));

  if (data_set_ptr->hasAttribute(HEADER_FRAME_ID))
  {
    std::string frameId;
    data_set_ptr->getAttribute(HEADER_FRAME_ID).read(frameId);
    return frameId;
  }
  else
  {
    return std::nullopt;
  }
}

void Hdf5CoreGeneral::readBoundingBoxLabeled(const std::string& datatypeGroup, const std::string& uuid,
                                             std::vector<std::string>& labels,
                                             std::vector<std::vector<double>>& boundingBoxes,
                                             std::vector<std::string>& instances, bool loadBoxes)
{
  std::string id = datatypeGroup + "/" + uuid;
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
      << "reading the bounding box 2d with labels of " << id;
  try
  {
    readLabel(id, LABELBB, labels);
    readInstances(id, LABELBBINSTANCES, instances);

    if (loadBoxes)
    {
      readBoundingBoxes(id, LABELBBBOXES, boundingBoxes);
    }
  }
  catch (std::invalid_argument const& e)
  {
    return;
  }

  if (labels.size() != instances.size() || (loadBoxes && labels.size() != boundingBoxes.size()))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "size of labels (" << labels.size() << "), size of bounding boxes (" << boundingBoxes.size()
        << ") and size of instances (" << instances.size() << ") do not fit.";
    labels.clear();
    boundingBoxes.clear();
    instances.clear();
    return;
  }
}

void Hdf5CoreGeneral::readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid,
                                        std::vector<std::string>& labels, std::vector<std::string>& instances)
{
  std::string id = datatypeGroup + "/" + uuid;
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "loading labels general of " << id;
  try
  {
    readLabel(id, LABELGENERAL, labels);
    readInstances(id, LABELGENERALINSTANCES, instances);
  }
  catch (std::invalid_argument const& e)
  {
    return;
  }

  if (labels.size() != instances.size())
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "size of labels (" << labels.size() << ") and size of instances (" << instances.size() << ") do not fit.";
    labels.clear();
    instances.clear();
    return;
  }
}

bool Hdf5CoreGeneral::hasTimeRaw(const std::string& datatypeGroup, const std::string& uuid)
{
  return hasTime(datatypeGroup, uuid + "/" + RAWDATA);
}

bool Hdf5CoreGeneral::hasTime(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  switch (m_file->getObjectType(id))
  {
    case HighFive::ObjectType::Group:
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
      return m_file->getGroup(id).hasAttribute(HEADER_STAMP_SECONDS) &&
             m_file->getGroup(id).hasAttribute(HEADER_STAMP_NANOS);

    case HighFive::ObjectType::Dataset:
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get dataset " << id;
      return m_file->getDataSet(id).hasAttribute(HEADER_STAMP_SECONDS) &&
             m_file->getDataSet(id).hasAttribute(HEADER_STAMP_NANOS);

    default:
      return false;
  }
}

void Hdf5CoreGeneral::writeTimeToRaw(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
                                     const int64_t& nanos)
{
  writeTime(datatypeGroup, uuid + "/" + RAWDATA, secs, nanos);
}

void Hdf5CoreGeneral::writeTime(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
                                const int64_t& nanos)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  switch (m_file->getObjectType(id))
  {
    case HighFive::ObjectType::Group:
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
      HighFive::Group group = m_file->getGroup(id);
      writeTimeToAnnotateTraits(secs, group, HEADER_STAMP_SECONDS);
      writeTimeToAnnotateTraits(nanos, group, HEADER_STAMP_NANOS);
    };
    break;

    case HighFive::ObjectType::Dataset:
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
      HighFive::DataSet dataset = m_file->getDataSet(id);
      writeTimeToAnnotateTraits(secs, dataset, HEADER_STAMP_SECONDS);
      writeTimeToAnnotateTraits(nanos, dataset, HEADER_STAMP_NANOS);
    };
    break;

    default:
      break;
  }
}

void Hdf5CoreGeneral::writeAABB(
    const std::string& datatypeGroup, const std::string& uuid,
    const boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
  HighFive::Group group = m_file->getGroup(id);

  std::vector<float> aabbPoints{ aabb.min_corner().get<0>(), aabb.min_corner().get<1>(), aabb.min_corner().get<2>(),
                                 aabb.max_corner().get<0>(), aabb.max_corner().get<1>(), aabb.max_corner().get<2>() };

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "write AABB as attribute";
  if (!group.hasAttribute(AABB_FIELD))
    group.createAttribute(AABB_FIELD, aabbPoints);
  else
    group.getAttribute(AABB_FIELD).write(aabbPoints);

  m_file->flush();
}

void Hdf5CoreGeneral::readAABB(
    const std::string& datatypeGroup, const std::string& uuid,
    boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
  HighFive::Group group = m_file->getGroup(id);
  if (group.hasAttribute(AABB_FIELD))
  {
    std::vector<float> aabbPoints;
    group.getAttribute(AABB_FIELD).read(aabbPoints);

    aabb.min_corner().set<0>(aabbPoints.at(0));
    aabb.min_corner().set<1>(aabbPoints.at(1));
    aabb.min_corner().set<2>(aabbPoints.at(2));
    aabb.max_corner().set<0>(aabbPoints.at(3));
    aabb.max_corner().set<1>(aabbPoints.at(4));
    aabb.max_corner().set<2>(aabbPoints.at(5));
  }
}

bool Hdf5CoreGeneral::hasAABB(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
  HighFive::Group group = m_file->getGroup(id);
  return group.hasAttribute(AABB_FIELD);
}

void Hdf5CoreGeneral::readTimeFromRaw(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs,
                                      int64_t& nanos)
{
  readTime(datatypeGroup, uuid + "/" + RAWDATA, secs, nanos);
}

void Hdf5CoreGeneral::readTime(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs, int64_t& nanos)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  switch (m_file->getObjectType(id))
  {
    case HighFive::ObjectType::Group:
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
      HighFive::Group group = m_file->getGroup(id);
      readTimeFromAnnotateTraits(id, secs, group, HEADER_STAMP_SECONDS);
      readTimeFromAnnotateTraits(id, nanos, group, HEADER_STAMP_NANOS);
    };
    break;

    case HighFive::ObjectType::Dataset:
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
      HighFive::DataSet dataset = m_file->getDataSet(id);
      readTimeFromAnnotateTraits(id, secs, dataset, HEADER_STAMP_SECONDS);
      readTimeFromAnnotateTraits(id, nanos, dataset, HEADER_STAMP_NANOS);
    };
    break;

    default:
      secs = std::numeric_limits<uint64_t>::min();
      nanos = std::numeric_limits<uint64_t>::min();
  }
}

void Hdf5CoreGeneral::deleteAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField)
{
  if (dataSetPtr->hasAttribute(attributeField))
  {
    dataSetPtr->deleteAttribute(attributeField);
    m_file->flush();
  }
}

void Hdf5CoreGeneral::checkExists(const std::string& id)
{
  if (!m_file->exist(id))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "id " << id << " does not exist in file " << m_file->getName();
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }
}

void Hdf5CoreGeneral::readLabel(const std::string& id, const std::string labelType, std::vector<std::string>& labels)
{
  checkExists(id + "/" + labelType);
  HighFive::DataSet datasetLabels = m_file->getDataSet(id + "/" + labelType);
  datasetLabels.read(labels);
}
void Hdf5CoreGeneral::readBoundingBoxes(const std::string& id, const std::string boundingBoxType,
                                        std::vector<std::vector<double>>& boundingBoxes)
{
  checkExists(id + "/" + boundingBoxType);
  HighFive::DataSet datasetBoxes = m_file->getDataSet(id + "/" + boundingBoxType);
  datasetBoxes.read(boundingBoxes);
}
void Hdf5CoreGeneral::readInstances(const std::string& id, const std::string instanceType,
                                    std::vector<std::string>& instances)
{
  checkExists(id + "/" + instanceType);
  HighFive::DataSet datasetInstances = m_file->getDataSet(id + "/" + instanceType);
  datasetInstances.read(instances);
}

std::shared_ptr<HighFive::Group> Hdf5CoreGeneral::getHdf5Group(const std::string& hdf5GroupPath, bool create)
{
  try
  {
    checkExists(hdf5GroupPath);
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "hdf5 group" << hdf5GroupPath << " already exists!";
    return std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));
  }
  catch (std::invalid_argument const& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "hdf5 group " << hdf5GroupPath << " does not exist! Creating a new group";
    if (create)
    {
      return std::make_shared<HighFive::Group>(m_file->createGroup(hdf5GroupPath));
    }
    else
    {
      return nullptr;
    }
  }
}

}  // namespace seerep_hdf5_core
