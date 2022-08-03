#include "seerep-hdf5-fb/hdf5-fb-general.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_fb
{
Hdf5FbGeneral::Hdf5FbGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : m_file(file), m_write_mtx(write_mtx)
{
}

std::optional<std::string> Hdf5FbGeneral::readFrameId(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  std::string hdf5DatasetRawDataPath = id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::RAWDATA;

  checkExists(hdf5DatasetRawDataPath);

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "get dataset " << hdf5DatasetRawDataPath;
  std::shared_ptr<HighFive::DataSet> data_set_ptr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRawDataPath));

  if (data_set_ptr->hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID))
  {
    std::string frameId;
    data_set_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID).read(frameId);
    return frameId;
  }
  else
  {
    return std::nullopt;
  }
}

std::vector<std::string> Hdf5FbGeneral::getGroupDatasets(const std::string& id)
{
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

void Hdf5FbGeneral::checkExists(const std::string& id)
{
  if (!m_file->exist(id))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "id " << id << " does not exist in file " << m_file->getName();
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }
}

void Hdf5FbGeneral::writeAttributeMap(
    const std::shared_ptr<HighFive::DataSet> dataSetPtr,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::UnionMapEntry>>& attributes)
{
  for (auto attribute : attributes)
  {
    if (attribute->value_type() == seerep::fb::Datatypes_Boolean)
    {
      writeAttribute(dataSetPtr, attribute->key()->str(),
                     static_cast<const seerep::fb::Boolean*>(attribute->value())->data());
    }
    else if (attribute->value_type() == seerep::fb::Datatypes_Integer)
    {
      writeAttribute(dataSetPtr, attribute->key()->str(),
                     static_cast<const seerep::fb::Integer*>(attribute->value())->data());
    }
    else if (attribute->value_type() == seerep::fb::Datatypes_Double)
    {
      writeAttribute(dataSetPtr, attribute->key()->str(),
                     static_cast<const seerep::fb::Double*>(attribute->value())->data());
    }
    else if (attribute->value_type() == seerep::fb::Datatypes_String)
    {
      writeAttribute(dataSetPtr, attribute->key()->str(),
                     static_cast<const seerep::fb::String*>(attribute->value())->data()->str());
    }
    else
    {
      std::stringstream errorMsg;
      errorMsg << "type " << attribute->value_type() << " of attribute " << attribute->key()->c_str()
               << " not implemented in hdf5-io.";
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << errorMsg.str();
      throw std::invalid_argument(errorMsg.str());
    }
  }
}

void Hdf5FbGeneral::deleteAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField)
{
  if (dataSetPtr->hasAttribute(attributeField))
  {
    dataSetPtr->deleteAttribute(attributeField);
    m_file->flush();
  }
}

void Hdf5FbGeneral::writeAABB(
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
  if (!group.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::AABB_FIELD))
    group.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::AABB_FIELD, aabbPoints);
  else
    group.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::AABB_FIELD).write(aabbPoints);

  m_file->flush();
}

void Hdf5FbGeneral::readAABB(
    const std::string& datatypeGroup, const std::string& uuid,
    boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
  HighFive::Group group = m_file->getGroup(id);
  if (group.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::AABB_FIELD))
  {
    std::vector<float> aabbPoints;
    group.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::AABB_FIELD).read(aabbPoints);

    aabb.min_corner().set<0>(aabbPoints.at(0));
    aabb.min_corner().set<1>(aabbPoints.at(1));
    aabb.min_corner().set<2>(aabbPoints.at(2));
    aabb.max_corner().set<0>(aabbPoints.at(3));
    aabb.max_corner().set<1>(aabbPoints.at(4));
    aabb.max_corner().set<2>(aabbPoints.at(5));
  }
}

bool Hdf5FbGeneral::hasAABB(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
  HighFive::Group group = m_file->getGroup(id);
  return group.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::AABB_FIELD);
}

void Hdf5FbGeneral::readTimeFromRaw(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs,
                                    int64_t& nanos)
{
  readTime(datatypeGroup, uuid + "/" + seerep_hdf5_core::Hdf5CoreGeneral::RAWDATA, secs, nanos);
}

void Hdf5FbGeneral::readTime(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs, int64_t& nanos)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  switch (m_file->getObjectType(id))
  {
    case HighFive::ObjectType::Group:
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
      HighFive::Group group = m_file->getGroup(id);
      readTimeFromAnnotateTraits(id, secs, group, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS);
      readTimeFromAnnotateTraits(id, nanos, group, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS);
    };
    break;

    case HighFive::ObjectType::Dataset:
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
      HighFive::DataSet dataset = m_file->getDataSet(id);
      readTimeFromAnnotateTraits(id, secs, dataset, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS);
      readTimeFromAnnotateTraits(id, nanos, dataset, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS);
    };
    break;

    default:
      secs = std::numeric_limits<uint64_t>::min();
      nanos = std::numeric_limits<uint64_t>::min();
  }
}

void Hdf5FbGeneral::writeTimeToRaw(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
                                   const int64_t& nanos)
{
  writeTime(datatypeGroup, uuid + "/" + seerep_hdf5_core::Hdf5CoreGeneral::RAWDATA, secs, nanos);
}

void Hdf5FbGeneral::writeTime(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
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
      writeTimeToAnnotateTraits(secs, group, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS);
      writeTimeToAnnotateTraits(nanos, group, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS);
    };
    break;

    case HighFive::ObjectType::Dataset:
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
      HighFive::DataSet dataset = m_file->getDataSet(id);
      writeTimeToAnnotateTraits(secs, dataset, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS);
      writeTimeToAnnotateTraits(nanos, dataset, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS);
    };
    break;

    default:
      break;
  }
}

bool Hdf5FbGeneral::hasTimeRaw(const std::string& datatypeGroup, const std::string& uuid)
{
  return hasTime(datatypeGroup, uuid + "/" + seerep_hdf5_core::Hdf5CoreGeneral::RAWDATA);
}

bool Hdf5FbGeneral::hasTime(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  switch (m_file->getObjectType(id))
  {
    case HighFive::ObjectType::Group:
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
      return m_file->getGroup(id).hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS) &&
             m_file->getGroup(id).hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS);

    case HighFive::ObjectType::Dataset:
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get dataset " << id;
      return m_file->getDataSet(id).hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS) &&
             m_file->getDataSet(id).hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS);

    default:
      return false;
  }
}

void Hdf5FbGeneral::writeBoundingBoxLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeled>>* boundingboxLabeled)
{
  if (boundingboxLabeled && !boundingboxLabeled->size() == 0)
  {
    std::string id = datatypeGroup + "/" + uuid;

    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    std::vector<std::string> instances;
    for (auto label : *boundingboxLabeled)
    {
      labels.push_back(label->labelWithInstance()->label()->str());
      std::vector<double> box{ label->bounding_box()->point_min()->x(), label->bounding_box()->point_min()->y(),
                               label->bounding_box()->point_min()->z(), label->bounding_box()->point_max()->x(),
                               label->bounding_box()->point_max()->y(), label->bounding_box()->point_max()->z() };
      boundingBoxes.push_back(box);

      instances.push_back(label->labelWithInstance()->instanceUuid()->str());
    }

    HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    HighFive::DataSet datasetBoxes = m_file->createDataSet<double>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES, HighFive::DataSpace::From(boundingBoxes));
    datasetBoxes.write(boundingBoxes);

    HighFive::DataSet datasetInstances = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES, HighFive::DataSpace::From(instances));
    datasetInstances.write(instances);

    m_file->flush();
  }
}

void Hdf5FbGeneral::writeBoundingBox2DLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeled>>* boundingbox2DLabeled)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (boundingbox2DLabeled && !boundingbox2DLabeled->size() == 0)
  {
    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    std::vector<std::string> instances;
    for (auto label : *boundingbox2DLabeled)
    {
      labels.push_back(label->labelWithInstance()->label()->str());
      std::vector<double> box{ label->bounding_box()->point_min()->x(), label->bounding_box()->point_min()->y(),
                               label->bounding_box()->point_max()->x(), label->bounding_box()->point_max()->y() };
      boundingBoxes.push_back(box);

      instances.push_back(label->labelWithInstance()->instanceUuid()->str());
    }

    HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    HighFive::DataSet datasetBoxes = m_file->createDataSet<double>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES, HighFive::DataSpace::From(boundingBoxes));
    datasetBoxes.write(boundingBoxes);

    HighFive::DataSet datasetInstances = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES, HighFive::DataSpace::From(instances));
    datasetInstances.write(instances);

    m_file->flush();
  }
}

void Hdf5FbGeneral::readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid,
                                             std::vector<std::string>& labels,
                                             std::vector<std::vector<double>>& boundingBoxes,
                                             std::vector<std::string>& instances)
{
  std::string id = datatypeGroup + "/" + uuid;
  try
  {
    checkExists(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB);
    checkExists(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES);
    checkExists(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES);
  }
  catch (std::invalid_argument const& e)
  {
    return;
  }

  HighFive::DataSet datasetLabels = m_file->getDataSet(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB);
  datasetLabels.read(labels);

  HighFive::DataSet datasetBoxes = m_file->getDataSet(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES);
  datasetBoxes.read(boundingBoxes);

  HighFive::DataSet datasetInstances =
      m_file->getDataSet(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES);
  datasetInstances.read(instances);

  if (labels.size() != boundingBoxes.size() || labels.size() != instances.size())
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

void Hdf5FbGeneral::writeLabelsGeneral(
    const std::string& datatypeGroup, const std::string& uuid,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::LabelWithInstance>>* labelsGeneral)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (labelsGeneral && !labelsGeneral->size() == 0)
  {
    std::vector<std::string> labels;
    std::vector<std::string> instances;
    for (auto label : *labelsGeneral)
    {
      labels.push_back(label->label()->str());

      instances.push_back(label->instanceUuid()->str());
    }

    HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    HighFive::DataSet datasetInstances = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES, HighFive::DataSpace::From(instances));
    datasetInstances.write(instances);

    m_file->flush();
  }
}

void Hdf5FbGeneral::readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid,
                                      std::vector<std::string>& labels, std::vector<std::string>& instances)
{
  std::string id = datatypeGroup + "/" + uuid;
  try
  {
    checkExists(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL);
    checkExists(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES);
  }
  catch (std::invalid_argument const& e)
  {
    return;
  }

  HighFive::DataSet datasetLabels = m_file->getDataSet(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL);
  datasetLabels.read(labels);

  HighFive::DataSet datasetInstances =
      m_file->getDataSet(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES);
  datasetInstances.read(instances);

  if (labels.size() != instances.size())
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "size of labels (" << labels.size() << ") and size of instances (" << instances.size() << ") do not fit.";
    labels.clear();
    instances.clear();
    return;
  }
}

void Hdf5FbGeneral::writeProjectname(const std::string& projectname)
{
  if (!m_file->hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::PROJECTNAME))
  {
    m_file->createAttribute<std::string>(seerep_hdf5_core::Hdf5CoreGeneral::PROJECTNAME, projectname);
  }
  else
  {
    m_file->getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::PROJECTNAME).write(projectname);
  }
  m_file->flush();
}

std::string Hdf5FbGeneral::readProjectname()
{
  std::string projectname;
  if (m_file->hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::PROJECTNAME))
  {
    m_file->getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::PROJECTNAME).read(projectname);
  }
  return projectname;
}

void Hdf5FbGeneral::writeProjectFrameId(const std::string& frameId)
{
  if (!m_file->hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::PROJECTFRAMEID))
  {
    m_file->createAttribute<std::string>(seerep_hdf5_core::Hdf5CoreGeneral::PROJECTFRAMEID, frameId);
  }
  else
  {
    m_file->getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::PROJECTFRAMEID).write(frameId);
  }
  m_file->flush();
}

std::string Hdf5FbGeneral::readProjectFrameId()
{
  std::string frameId;
  if (m_file->hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::PROJECTFRAMEID))
  {
    m_file->getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::PROJECTFRAMEID).read(frameId);
  }
  return frameId;
}

}  // namespace seerep_hdf5_fb
