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
  std::string hdf5DatasetRawDataPath = id + "/" + RAWDATA;
  if (!m_file->exist(hdf5DatasetRawDataPath))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "id " << hdf5DatasetRawDataPath << " does not exist in file " << m_file->getName();
    throw std::invalid_argument("id " + hdf5DatasetRawDataPath + " does not exist in file " + m_file->getName());
  }
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
  if (!m_file->exist(id))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "id " << id << " does not exist in file " << m_file->getName();
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }
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

void Hdf5FbGeneral::readAABB(
    const std::string& datatypeGroup, const std::string& uuid,
    boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!m_file->exist(id))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "id " << id << " does not exist in file " << m_file->getName();
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }
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

bool Hdf5FbGeneral::hasAABB(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!m_file->exist(id))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "id " << id << " does not exist in file " << m_file->getName();
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
  HighFive::Group group = m_file->getGroup(id);
  return group.hasAttribute(AABB_FIELD);
}

void Hdf5FbGeneral::readTimeFromRaw(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs,
                                    int64_t& nanos)
{
  readTime(datatypeGroup, uuid + "/" + RAWDATA, secs, nanos);
}

void Hdf5FbGeneral::readTime(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs, int64_t& nanos)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!m_file->exist(id))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "id " << id << " does not exist in file " << m_file->getName();
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }

  switch (m_file->getObjectType(id))
  {
    case HighFive::ObjectType::Group:
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
      HighFive::Group group = m_file->getGroup(id);
      if (group.hasAttribute(HEADER_STAMP_SECONDS))
      {
        group.getAttribute(HEADER_STAMP_SECONDS).read(secs);
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " + HEADER_STAMP_SECONDS);
      }
      if (group.hasAttribute(HEADER_STAMP_NANOS))
      {
        group.getAttribute(HEADER_STAMP_NANOS).read(nanos);
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " + HEADER_STAMP_NANOS);
      }
    };
    break;

    case HighFive::ObjectType::Dataset:
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
      HighFive::DataSet dataset = m_file->getDataSet(id);
      if (dataset.hasAttribute(HEADER_STAMP_SECONDS))
      {
        dataset.getAttribute(HEADER_STAMP_SECONDS).read(secs);
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " + HEADER_STAMP_SECONDS);
      }
      if (dataset.hasAttribute(HEADER_STAMP_NANOS))
      {
        dataset.getAttribute(HEADER_STAMP_NANOS).read(nanos);
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " + HEADER_STAMP_NANOS);
      }
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
  writeTime(datatypeGroup, uuid + "/" + RAWDATA, secs, nanos);
}

void Hdf5FbGeneral::writeTime(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
                              const int64_t& nanos)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!m_file->exist(id))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "id " << id << " does not exist in file " << m_file->getName();
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }

  switch (m_file->getObjectType(id))
  {
    case HighFive::ObjectType::Group:
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
      HighFive::Group group = m_file->getGroup(id);
      if (group.hasAttribute(HEADER_STAMP_SECONDS))
      {
        group.getAttribute(HEADER_STAMP_SECONDS).write(secs);
      }
      else
      {
        group.createAttribute(HEADER_STAMP_SECONDS, secs);
      }
      if (group.hasAttribute(HEADER_STAMP_NANOS))
      {
        group.getAttribute(HEADER_STAMP_NANOS).write(nanos);
      }
      else
      {
        group.createAttribute(HEADER_STAMP_NANOS, nanos);
      }
      m_file->flush();
      return;
    };

    case HighFive::ObjectType::Dataset:
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "get group " << id;
      HighFive::DataSet dataset = m_file->getDataSet(id);
      if (dataset.hasAttribute(HEADER_STAMP_SECONDS))
      {
        dataset.getAttribute(HEADER_STAMP_SECONDS).write(secs);
      }
      else
      {
        dataset.createAttribute(HEADER_STAMP_SECONDS, secs);
      }
      if (dataset.hasAttribute(HEADER_STAMP_NANOS))
      {
        dataset.getAttribute(HEADER_STAMP_NANOS).write(nanos);
      }
      else
      {
        dataset.createAttribute(HEADER_STAMP_NANOS, nanos);
      }
      m_file->flush();
      return;
    };
    default:
      return;
  }
}

bool Hdf5FbGeneral::hasTimeRaw(const std::string& datatypeGroup, const std::string& uuid)
{
  return hasTime(datatypeGroup, uuid + "/" + RAWDATA);
}

bool Hdf5FbGeneral::hasTime(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!m_file->exist(id))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "id " << id << " does not exist in file " << m_file->getName();
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }

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

void Hdf5FbGeneral::writeBoundingBoxLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeled>>* boundingboxLabeled)
{
  if (boundingboxLabeled && !boundingboxLabeled->size() == 0)
  {
    std::string id = datatypeGroup + "/" + uuid;

    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    for (auto label : *boundingboxLabeled)
    {
      ///@todo write instance
      labels.push_back(label->labelWithInstance()->label()->str());
      std::vector<double> box{ label->bounding_box()->point_min()->x(), label->bounding_box()->point_min()->y(),
                               label->bounding_box()->point_min()->z(), label->bounding_box()->point_max()->x(),
                               label->bounding_box()->point_max()->y(), label->bounding_box()->point_max()->z() };
      boundingBoxes.push_back(box);
    }

    HighFive::DataSet datasetLabels =
        m_file->createDataSet<std::string>(id + "/" + LABELBB, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    HighFive::DataSet datasetBoxes =
        m_file->createDataSet<double>(id + "/" + LABELBBBOXES, HighFive::DataSpace::From(boundingBoxes));
    datasetBoxes.write(boundingBoxes);

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
    for (auto label : *boundingbox2DLabeled)
    {
      ///@todo write instace
      labels.push_back(label->labelWithInstance()->label()->str());
      std::vector<double> box{ label->bounding_box()->point_min()->x(), label->bounding_box()->point_min()->y(),
                               label->bounding_box()->point_max()->x(), label->bounding_box()->point_max()->y() };
      boundingBoxes.push_back(box);
    }

    HighFive::DataSet datasetLabels =
        m_file->createDataSet<std::string>(id + "/" + LABELBB, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    HighFive::DataSet datasetBoxes =
        m_file->createDataSet<double>(id + "/" + LABELBBBOXES, HighFive::DataSpace::From(boundingBoxes));
    datasetBoxes.write(boundingBoxes);

    m_file->flush();
  }
}

void Hdf5FbGeneral::readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid,
                                             std::vector<std::string>& labels,
                                             std::vector<std::vector<double>>& boundingBoxes)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!m_file->exist(id + "/" + LABELBB))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "id " << id + "/" + LABELBB << " does not exist in file " << m_file->getName();
    return;
  }
  if (!m_file->exist(id + "/" + LABELBBBOXES))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "id " << id + "/" + LABELBBBOXES << " does not exist in file " << m_file->getName();
    return;
  }

  HighFive::DataSet datasetLabels = m_file->getDataSet(id + "/" + LABELBB);
  datasetLabels.read(labels);

  HighFive::DataSet datasetBoxes = m_file->getDataSet(id + "/" + LABELBBBOXES);
  datasetBoxes.read(boundingBoxes);

  if (labels.size() != boundingBoxes.size())
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "size of labels (" << labels.size() << ") and size of bounding boxes (" << boundingBoxes.size()
        << ") do not fit.";
    labels.clear();
    boundingBoxes.clear();
    return;
  }

  // flatbuffers::Vector<seerep::fb::BoundingBox2DLabeled> result;

  // for (int i = 0; i < labels.size(); i++)
  // {
  //   seerep::fb::BoundingBox2DLabeled bblabeled;
  //   bblabeled.set_label(labels.at(i));

  //   bblabeled.mutable_boundingbox()->mutable_point_min()->set_x(boundingBoxes.at(i).at(0));
  //   bblabeled.mutable_boundingbox()->mutable_point_min()->set_y(boundingBoxes.at(i).at(1));
  //   bblabeled.mutable_boundingbox()->mutable_point_max()->set_x(boundingBoxes.at(i).at(2));
  //   bblabeled.mutable_boundingbox()->mutable_point_max()->set_y(boundingBoxes.at(i).at(3));

  //   result.Add(std::move(bblabeled));
  // }

  // return result;
}

void Hdf5FbGeneral::writeLabelsGeneral(
    const std::string& datatypeGroup, const std::string& uuid,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::LabelWithInstance>>* labelsGeneral)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (labelsGeneral && !labelsGeneral->size() == 0)
  {
    std::vector<std::string> labels;
    for (auto label : *labelsGeneral)
    {
      ///@todo write instance
      labels.push_back(label->label()->str());
    }

    HighFive::DataSet datasetLabels =
        m_file->createDataSet<std::string>(id + "/" + LABELGENERAL, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    m_file->flush();
  }
}

void Hdf5FbGeneral::readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid,
                                      std::vector<std::string>& labels)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!m_file->exist(id + "/" + LABELGENERAL))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "id " << id + "/" + LABELGENERAL << " does not exist in file " << m_file->getName();
    return;
  }

  HighFive::DataSet datasetLabels = m_file->getDataSet(id + "/" + LABELGENERAL);
  datasetLabels.read(labels);

  // google::protobuf::RepeatedPtrField<std::string> result;

  // for (int i = 0; i < labels.size(); i++)
  // {
  //   result.Add(std::move(labels.at(i)));
  // }

  // return result;
}

void Hdf5FbGeneral::writeProjectname(const std::string& projectname)
{
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

std::string Hdf5FbGeneral::readProjectname()
{
  std::string projectname;
  if (m_file->hasAttribute(PROJECTNAME))
  {
    m_file->getAttribute(PROJECTNAME).read(projectname);
  }
  return projectname;
}

void Hdf5FbGeneral::writeProjectFrameId(const std::string& frameId)
{
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

std::string Hdf5FbGeneral::readProjectFrameId()
{
  std::string frameId;
  if (m_file->hasAttribute(PROJECTFRAMEID))
  {
    m_file->getAttribute(PROJECTFRAMEID).read(frameId);
  }
  return frameId;
}

}  // namespace seerep_hdf5_fb
