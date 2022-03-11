#include "seerep-io-pb/io-pb-general.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_io_pb
{
IoPbGeneral::IoPbGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : m_file(file), m_write_mtx(write_mtx)
{
}

std::optional<std::string> IoPbGeneral::readFrameId(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  std::string hdf5DatasetRawDataPath = id + "/" + seerep_io_core::IoCoreGeneral::RAWDATA;
  if (!m_file->exist(hdf5DatasetRawDataPath))
  {
    std::cout << "id " << hdf5DatasetRawDataPath << " does not exist in file " << m_file->getName() << std::endl;
    throw std::invalid_argument("id " + hdf5DatasetRawDataPath + " does not exist in file " + m_file->getName());
  }
  std::cout << "get dataset " << hdf5DatasetRawDataPath << std::endl;
  std::shared_ptr<HighFive::DataSet> data_set_ptr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRawDataPath));

  if (data_set_ptr->hasAttribute(seerep_io_core::IoCoreGeneral::HEADER_FRAME_ID))
  {
    std::string frameId;
    data_set_ptr->getAttribute(seerep_io_core::IoCoreGeneral::HEADER_FRAME_ID).read(frameId);
    return frameId;
  }
  else
  {
    return std::nullopt;
  }
}

std::vector<std::string> IoPbGeneral::getGroupDatasets(const std::string& id)
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

void IoPbGeneral::deleteAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField)
{
  if (dataSetPtr->hasAttribute(attributeField))
  {
    dataSetPtr->deleteAttribute(attributeField);
    m_file->flush();
  }
}

void IoPbGeneral::writeAABB(
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
  if (!group.hasAttribute(seerep_io_core::IoCoreGeneral::AABB_FIELD))
    group.createAttribute(seerep_io_core::IoCoreGeneral::AABB_FIELD, aabbPoints);
  else
    group.getAttribute(seerep_io_core::IoCoreGeneral::AABB_FIELD).write(aabbPoints);

  m_file->flush();
}

void IoPbGeneral::readAABB(
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
  if (group.hasAttribute(seerep_io_core::IoCoreGeneral::AABB_FIELD))
  {
    std::vector<float> aabbPoints;
    group.getAttribute(seerep_io_core::IoCoreGeneral::AABB_FIELD).read(aabbPoints);

    aabb.min_corner().set<0>(aabbPoints.at(0));
    aabb.min_corner().set<1>(aabbPoints.at(1));
    aabb.min_corner().set<2>(aabbPoints.at(2));
    aabb.max_corner().set<0>(aabbPoints.at(3));
    aabb.max_corner().set<1>(aabbPoints.at(4));
    aabb.max_corner().set<2>(aabbPoints.at(5));
  }
}

bool IoPbGeneral::hasAABB(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!m_file->exist(id))
  {
    std::cout << "id " << id << " does not exist in file " << m_file->getName() << std::endl;
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }
  std::cout << "get group " << id << std::endl;
  HighFive::Group group = m_file->getGroup(id);
  return group.hasAttribute(seerep_io_core::IoCoreGeneral::AABB_FIELD);
}

void IoPbGeneral::readTimeFromRaw(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs,
                                  int64_t& nanos)
{
  readTime(datatypeGroup, uuid + "/" + seerep_io_core::IoCoreGeneral::RAWDATA, secs, nanos);
}

void IoPbGeneral::readTime(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs, int64_t& nanos)
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
      if (group.hasAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS))
      {
        group.getAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS).read(secs);
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " +
                                    seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS);
      }
      if (group.hasAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS))
      {
        group.getAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS).read(nanos);
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " +
                                    seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS);
      }
    };
    break;

    case HighFive::ObjectType::Dataset:
    {
      std::cout << "get group " << id << std::endl;
      HighFive::DataSet dataset = m_file->getDataSet(id);
      if (dataset.hasAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS))
      {
        dataset.getAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS).read(secs);
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " +
                                    seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS);
      }
      if (dataset.hasAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS))
      {
        dataset.getAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS).read(nanos);
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " +
                                    seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS);
      }
    };
    break;

    default:
      secs = std::numeric_limits<uint64_t>::min();
      nanos = std::numeric_limits<uint64_t>::min();
  }
}

void IoPbGeneral::writeTimeToRaw(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
                                 const int64_t& nanos)
{
  writeTime(datatypeGroup, uuid + "/" + seerep_io_core::IoCoreGeneral::RAWDATA, secs, nanos);
}

void IoPbGeneral::writeTime(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
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
      if (group.hasAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS))
      {
        group.getAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS).write(secs);
      }
      else
      {
        group.createAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS, secs);
      }
      if (group.hasAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS))
      {
        group.getAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS).write(nanos);
      }
      else
      {
        group.createAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS, nanos);
      }
      m_file->flush();
      return;
    };

    case HighFive::ObjectType::Dataset:
    {
      std::cout << "get group " << id << std::endl;
      HighFive::DataSet dataset = m_file->getDataSet(id);
      if (dataset.hasAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS))
      {
        dataset.getAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS).write(secs);
      }
      else
      {
        dataset.createAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS, secs);
      }
      if (dataset.hasAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS))
      {
        dataset.getAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS).write(nanos);
      }
      else
      {
        dataset.createAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS, nanos);
      }
      m_file->flush();
      return;
    };
    default:
      return;
  }
}

bool IoPbGeneral::hasTimeRaw(const std::string& datatypeGroup, const std::string& uuid)
{
  return hasTime(datatypeGroup, uuid + "/" + seerep_io_core::IoCoreGeneral::RAWDATA);
}

bool IoPbGeneral::hasTime(const std::string& datatypeGroup, const std::string& uuid)
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
      return m_file->getGroup(id).hasAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS) &&
             m_file->getGroup(id).hasAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS);

    case HighFive::ObjectType::Dataset:
      std::cout << "get dataset " << id << std::endl;
      return m_file->getDataSet(id).hasAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS) &&
             m_file->getDataSet(id).hasAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS);

    default:
      return false;
  }
}

void IoPbGeneral::writeBoundingBoxLabeled(
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

    HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
        id + "/" + seerep_io_core::IoCoreGeneral::LABELBB, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    HighFive::DataSet datasetBoxes = m_file->createDataSet<double>(
        id + "/" + seerep_io_core::IoCoreGeneral::LABELBBBOXES, HighFive::DataSpace::From(boundingBoxes));
    datasetBoxes.write(boundingBoxes);

    m_file->flush();
  }
}

void IoPbGeneral::writeBoundingBox2DLabeled(
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

    HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
        id + "/" + seerep_io_core::IoCoreGeneral::LABELBB, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    HighFive::DataSet datasetBoxes = m_file->createDataSet<double>(
        id + "/" + seerep_io_core::IoCoreGeneral::LABELBBBOXES, HighFive::DataSpace::From(boundingBoxes));
    datasetBoxes.write(boundingBoxes);

    m_file->flush();
  }
}

std::optional<google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeled>>
IoPbGeneral::readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!m_file->exist(id + "/" + seerep_io_core::IoCoreGeneral::LABELBB))
  {
    std::cout << "id " << id + "/" + seerep_io_core::IoCoreGeneral::LABELBB << " does not exist in file "
              << m_file->getName() << std::endl;
    return std::nullopt;
  }
  if (!m_file->exist(id + "/" + seerep_io_core::IoCoreGeneral::LABELBBBOXES))
  {
    std::cout << "id " << id + "/" + seerep_io_core::IoCoreGeneral::LABELBBBOXES << " does not exist in file "
              << m_file->getName() << std::endl;
    return std::nullopt;
  }

  std::vector<std::string> labels;
  std::vector<std::vector<double>> boundingBoxes;

  HighFive::DataSet datasetLabels = m_file->getDataSet(id + "/" + seerep_io_core::IoCoreGeneral::LABELBB);
  datasetLabels.read(labels);

  HighFive::DataSet datasetBoxes = m_file->getDataSet(id + "/" + seerep_io_core::IoCoreGeneral::LABELBBBOXES);
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

void IoPbGeneral::writeLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid,
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

    HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
        id + "/" + seerep_io_core::IoCoreGeneral::LABELGENERAL, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    m_file->flush();
  }
}

std::optional<google::protobuf::RepeatedPtrField<std::string>>
IoPbGeneral::readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!m_file->exist(id + "/" + seerep_io_core::IoCoreGeneral::LABELGENERAL))
  {
    std::cout << "id " << id + "/" + seerep_io_core::IoCoreGeneral::LABELGENERAL << " does not exist in file "
              << m_file->getName() << std::endl;
    return std::nullopt;
  }

  std::vector<std::string> labels;
  HighFive::DataSet datasetLabels = m_file->getDataSet(id + "/" + seerep_io_core::IoCoreGeneral::LABELGENERAL);
  datasetLabels.read(labels);

  google::protobuf::RepeatedPtrField<std::string> result;

  for (int i = 0; i < labels.size(); i++)
  {
    result.Add(std::move(labels.at(i)));
  }

  return result;
}

void IoPbGeneral::writeProjectname(const std::string& projectname)
{
  if (!m_file->hasAttribute(seerep_io_core::IoCoreGeneral::PROJECTNAME))
  {
    m_file->createAttribute<std::string>(seerep_io_core::IoCoreGeneral::PROJECTNAME, projectname);
  }
  else
  {
    m_file->getAttribute(seerep_io_core::IoCoreGeneral::PROJECTNAME).write(projectname);
  }
  m_file->flush();
}

std::string IoPbGeneral::readProjectname()
{
  std::string projectname;
  if (m_file->hasAttribute(seerep_io_core::IoCoreGeneral::PROJECTNAME))
  {
    m_file->getAttribute(seerep_io_core::IoCoreGeneral::PROJECTNAME).read(projectname);
  }
  return projectname;
}

void IoPbGeneral::writeProjectFrameId(const std::string& frameId)
{
  if (!m_file->hasAttribute(seerep_io_core::IoCoreGeneral::PROJECTFRAMEID))
  {
    m_file->createAttribute<std::string>(seerep_io_core::IoCoreGeneral::PROJECTFRAMEID, frameId);
  }
  else
  {
    m_file->getAttribute(seerep_io_core::IoCoreGeneral::PROJECTFRAMEID).write(frameId);
  }
  m_file->flush();
}

std::string IoPbGeneral::readProjectFrameId()
{
  std::string frameId;
  if (m_file->hasAttribute(seerep_io_core::IoCoreGeneral::PROJECTFRAMEID))
  {
    m_file->getAttribute(seerep_io_core::IoCoreGeneral::PROJECTFRAMEID).read(frameId);
  }
  return frameId;
}

} /* namespace seerep_io_pb */
