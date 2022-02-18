#include "seerep-hdf5/ioGeneral.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5
{
SeerepHDF5IOGeneral::SeerepHDF5IOGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : m_file(file), m_write_mtx(write_mtx)
{
}

std::optional<std::string> SeerepHDF5IOGeneral::readFrameId(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  std::string hdf5DatasetRawDataPath = id + "/" + RAWDATA;
  if (!m_file->exist(hdf5DatasetRawDataPath))
  {
    std::cout << "id " << hdf5DatasetRawDataPath << " does not exist in file " << m_file->getName() << std::endl;
    throw std::invalid_argument("id " + hdf5DatasetRawDataPath + " does not exist in file " + m_file->getName());
  }
  std::cout << "get dataset " << hdf5DatasetRawDataPath << std::endl;
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

std::vector<std::string> SeerepHDF5IOGeneral::getGroupDatasets(const std::string& id)
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

template <typename T>
void SeerepHDF5IOGeneral::writeAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr,
                                         std::string attributeField, T value)
{
  if (!dataSetPtr->hasAttribute(attributeField))
  {
    dataSetPtr->createAttribute(attributeField, value);
  }
  else
  {
    dataSetPtr->getAttribute(attributeField).write(value);
  }
  m_file->flush();
}

template <typename T>
T SeerepHDF5IOGeneral::getAttribute(const std::string& id, const std::shared_ptr<HighFive::DataSet> dataSetPtr,
                                    std::string attributeField)
{
  T attributeValue;
  if (dataSetPtr->hasAttribute(attributeField))
  {
    dataSetPtr->getAttribute(attributeField).read(attributeValue);
  }
  else
  {
    throw std::invalid_argument("id " + id + " has no attribute " + attributeField);
  }
  return attributeValue;
}

void SeerepHDF5IOGeneral::deleteAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr,
                                          std::string attributeField)
{
  if (dataSetPtr->hasAttribute(attributeField))
  {
    dataSetPtr->deleteAttribute(attributeField);
    m_file->flush();
  }
}

template <class T>
void SeerepHDF5IOGeneral::writeHeaderAttributes(HighFive::AnnotateTraits<T>& object, const seerep::Header& header)
{
  if (!object.hasAttribute(HEADER_STAMP_SECONDS))
    object.createAttribute(HEADER_STAMP_SECONDS, header.stamp().seconds());
  else
    object.getAttribute(HEADER_STAMP_SECONDS).write(header.stamp().seconds());

  if (!object.hasAttribute(HEADER_STAMP_NANOS))
    object.createAttribute(HEADER_STAMP_NANOS, header.stamp().nanos());
  else
    object.getAttribute(HEADER_STAMP_NANOS).write(header.stamp().nanos());

  if (!object.hasAttribute(HEADER_FRAME_ID))
    object.createAttribute(HEADER_FRAME_ID, header.frame_id());
  else
    object.getAttribute(HEADER_FRAME_ID).write(header.frame_id());

  if (!object.hasAttribute(HEADER_SEQ))
    object.createAttribute(HEADER_SEQ, header.seq());
  else
    object.getAttribute(HEADER_SEQ).write(header.seq());
}

template <class T>
seerep::Header SeerepHDF5IOGeneral::readHeaderAttributes(HighFive::AnnotateTraits<T>& object)
{
  seerep::Header header;

  int64_t seconds;
  int32_t nanos;
  uint32_t seq;

  object.getAttribute(HEADER_FRAME_ID).read(header.mutable_frame_id());

  object.getAttribute(HEADER_STAMP_SECONDS).read(seconds);
  object.getAttribute(HEADER_STAMP_NANOS).read(nanos);
  object.getAttribute(HEADER_SEQ).read(seq);

  header.set_seq(seq);
  header.mutable_stamp()->set_seconds(seconds);
  header.mutable_stamp()->set_nanos(nanos);

  return header;
}

void SeerepHDF5IOGeneral::writeAABB(
    const std::string& datatypeGroup, const std::string& uuid,
    const boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!m_file->exist(id))
  {
    std::cout << "id " << id << " does not exist in file " << m_file->getName() << std::endl;
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }
  std::cout << "get group " << id << std::endl;
  HighFive::Group group = m_file->getGroup(id);

  std::vector<float> aabbPoints{ aabb.min_corner().get<0>(), aabb.min_corner().get<1>(), aabb.min_corner().get<2>(),
                                 aabb.max_corner().get<0>(), aabb.max_corner().get<1>(), aabb.max_corner().get<2>() };

  std::cout << "write AABB as attribute" << std::endl;
  if (!group.hasAttribute(AABB_FIELD))
    group.createAttribute(AABB_FIELD, aabbPoints);
  else
    group.getAttribute(AABB_FIELD).write(aabbPoints);

  m_file->flush();
}

void SeerepHDF5IOGeneral::readAABB(
    const std::string& datatypeGroup, const std::string& uuid,
    boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!m_file->exist(id))
  {
    std::cout << "id " << id << " does not exist in file " << m_file->getName() << std::endl;
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }
  std::cout << "get group " << id << std::endl;
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

bool SeerepHDF5IOGeneral::hasAABB(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!m_file->exist(id))
  {
    std::cout << "id " << id << " does not exist in file " << m_file->getName() << std::endl;
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }
  std::cout << "get group " << id << std::endl;
  HighFive::Group group = m_file->getGroup(id);
  return group.hasAttribute(AABB_FIELD);
}

void SeerepHDF5IOGeneral::readTimeFromRaw(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs,
                                          int64_t& nanos)
{
  readTime(datatypeGroup, uuid + "/" + RAWDATA, secs, nanos);
}

void SeerepHDF5IOGeneral::readTime(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs,
                                   int64_t& nanos)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!m_file->exist(id))
  {
    std::cout << "id " << id << " does not exist in file " << m_file->getName() << std::endl;
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }

  switch (m_file->getObjectType(id))
  {
    case HighFive::ObjectType::Group:
    {
      std::cout << "get group " << id << std::endl;
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
      std::cout << "get group " << id << std::endl;
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

void SeerepHDF5IOGeneral::writeTimeToRaw(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
                                         const int64_t& nanos)
{
  writeTime(datatypeGroup, uuid + "/" + RAWDATA, secs, nanos);
}

void SeerepHDF5IOGeneral::writeTime(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
                                    const int64_t& nanos)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!m_file->exist(id))
  {
    std::cout << "id " << id << " does not exist in file " << m_file->getName() << std::endl;
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }

  switch (m_file->getObjectType(id))
  {
    case HighFive::ObjectType::Group:
    {
      std::cout << "get group " << id << std::endl;
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
      std::cout << "get group " << id << std::endl;
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

bool SeerepHDF5IOGeneral::hasTimeRaw(const std::string& datatypeGroup, const std::string& uuid)
{
  return hasTime(datatypeGroup, uuid + "/" + RAWDATA);
}

bool SeerepHDF5IOGeneral::hasTime(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!m_file->exist(id))
  {
    std::cout << "id " << id << " does not exist in file " << m_file->getName() << std::endl;
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }

  switch (m_file->getObjectType(id))
  {
    case HighFive::ObjectType::Group:
      std::cout << "get group " << id << std::endl;
      return m_file->getGroup(id).hasAttribute(HEADER_STAMP_SECONDS) &&
             m_file->getGroup(id).hasAttribute(HEADER_STAMP_NANOS);

    case HighFive::ObjectType::Dataset:
      std::cout << "get dataset " << id << std::endl;
      return m_file->getDataSet(id).hasAttribute(HEADER_STAMP_SECONDS) &&
             m_file->getDataSet(id).hasAttribute(HEADER_STAMP_NANOS);

    default:
      return false;
  }
}

void SeerepHDF5IOGeneral::writeBoundingBoxLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const google::protobuf::RepeatedPtrField<::seerep::BoundingBoxLabeled>& boundingboxLabeled)
{
  if (!boundingboxLabeled.empty())
  {
    std::string id = datatypeGroup + "/" + uuid;

    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    for (auto label : boundingboxLabeled)
    {
      labels.push_back(label.label());
      std::vector<double> box{ label.boundingbox().point_min().x(), label.boundingbox().point_min().y(),
                               label.boundingbox().point_min().z(), label.boundingbox().point_max().x(),
                               label.boundingbox().point_max().y(), label.boundingbox().point_max().z() };
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

void SeerepHDF5IOGeneral::writeBoundingBox2DLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const google::protobuf::RepeatedPtrField<::seerep::BoundingBox2DLabeled>& boundingbox2DLabeled)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!boundingbox2DLabeled.empty())
  {
    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    for (auto label : boundingbox2DLabeled)
    {
      labels.push_back(label.label());
      std::vector<double> box{ label.boundingbox().point_min().x(), label.boundingbox().point_min().y(),
                               label.boundingbox().point_max().x(), label.boundingbox().point_max().y() };
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

std::optional<google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeled>>
SeerepHDF5IOGeneral::readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!m_file->exist(id + "/" + LABELBB))
  {
    std::cout << "id " << id + "/" + LABELBB << " does not exist in file " << m_file->getName() << std::endl;
    return std::nullopt;
  }
  if (!m_file->exist(id + "/" + LABELBBBOXES))
  {
    std::cout << "id " << id + "/" + LABELBBBOXES << " does not exist in file " << m_file->getName() << std::endl;
    return std::nullopt;
  }

  std::vector<std::string> labels;
  std::vector<std::vector<double>> boundingBoxes;

  HighFive::DataSet datasetLabels = m_file->getDataSet(id + "/" + LABELBB);
  datasetLabels.read(labels);

  HighFive::DataSet datasetBoxes = m_file->getDataSet(id + "/" + LABELBBBOXES);
  datasetBoxes.read(boundingBoxes);

  if (labels.size() != boundingBoxes.size())
  {
    std::cout << "size of labels (" << labels.size() << ") and size of bounding boxes (" << boundingBoxes.size()
              << ") do not fit." << std::endl;
    return std::nullopt;
  }

  google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeled> result;

  for (int i = 0; i < labels.size(); i++)
  {
    seerep::BoundingBox2DLabeled bblabeled;
    bblabeled.set_label(labels.at(i));

    bblabeled.mutable_boundingbox()->mutable_point_min()->set_x(boundingBoxes.at(i).at(0));
    bblabeled.mutable_boundingbox()->mutable_point_min()->set_y(boundingBoxes.at(i).at(1));
    bblabeled.mutable_boundingbox()->mutable_point_max()->set_x(boundingBoxes.at(i).at(2));
    bblabeled.mutable_boundingbox()->mutable_point_max()->set_y(boundingBoxes.at(i).at(3));

    result.Add(std::move(bblabeled));
  }

  return result;
}

void SeerepHDF5IOGeneral::writeLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid,
                                             const google::protobuf::RepeatedPtrField<std::string>& labelsGeneral)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!labelsGeneral.empty())
  {
    std::vector<std::string> labels;
    for (auto label : labelsGeneral)
    {
      labels.push_back(label);
    }

    HighFive::DataSet datasetLabels =
        m_file->createDataSet<std::string>(id + "/" + LABELGENERAL, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    m_file->flush();
  }
}

std::optional<google::protobuf::RepeatedPtrField<std::string>>
SeerepHDF5IOGeneral::readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!m_file->exist(id + "/" + LABELGENERAL))
  {
    std::cout << "id " << id + "/" + LABELGENERAL << " does not exist in file " << m_file->getName() << std::endl;
    return std::nullopt;
  }

  std::vector<std::string> labels;
  HighFive::DataSet datasetLabels = m_file->getDataSet(id + "/" + LABELGENERAL);
  datasetLabels.read(labels);

  google::protobuf::RepeatedPtrField<std::string> result;

  for (int i = 0; i < labels.size(); i++)
  {
    result.Add(std::move(labels.at(i)));
  }

  return result;
}

void SeerepHDF5IOGeneral::writeProjectname(const std::string& projectname)
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

std::string SeerepHDF5IOGeneral::readProjectname()
{
  std::string projectname;
  if (m_file->hasAttribute(PROJECTNAME))
  {
    m_file->getAttribute(PROJECTNAME).read(projectname);
  }
  return projectname;
}

void SeerepHDF5IOGeneral::writeProjectFrameId(const std::string& frameId)
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

std::string SeerepHDF5IOGeneral::readProjectFrameId()
{
  std::string frameId;
  if (m_file->hasAttribute(PROJECTFRAMEID))
  {
    m_file->getAttribute(PROJECTFRAMEID).read(frameId);
  }
  return frameId;
}

} /* namespace seerep_hdf5 */
