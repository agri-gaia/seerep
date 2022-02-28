#include "seerep-hdf5-flatbuffers/general-io-fbs.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5
{
GeneralIOFbs::GeneralIOFbs(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : m_file(file), m_write_mtx(write_mtx)
{
}

std::optional<std::string> GeneralIOFbs::readFrameId(const std::string& datatypeGroup, const std::string& uuid)
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

std::vector<std::string> GeneralIOFbs::getGroupDatasets(const std::string& id)
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

void GeneralIOFbs::deleteAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField)
{
  if (dataSetPtr->hasAttribute(attributeField))
  {
    dataSetPtr->deleteAttribute(attributeField);
    m_file->flush();
  }
}

void GeneralIOFbs::writeAABB(
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

void GeneralIOFbs::readAABB(
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

bool GeneralIOFbs::hasAABB(const std::string& datatypeGroup, const std::string& uuid)
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

void GeneralIOFbs::readTimeFromRaw(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs,
                                   int64_t& nanos)
{
  readTime(datatypeGroup, uuid + "/" + RAWDATA, secs, nanos);
}

void GeneralIOFbs::readTime(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs, int64_t& nanos)
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

void GeneralIOFbs::writeTimeToRaw(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
                                  const int64_t& nanos)
{
  writeTime(datatypeGroup, uuid + "/" + RAWDATA, secs, nanos);
}

void GeneralIOFbs::writeTime(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
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

bool GeneralIOFbs::hasTimeRaw(const std::string& datatypeGroup, const std::string& uuid)
{
  return hasTime(datatypeGroup, uuid + "/" + RAWDATA);
}

bool GeneralIOFbs::hasTime(const std::string& datatypeGroup, const std::string& uuid)
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

void GeneralIOFbs::writeBoundingBoxLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeled>>& boundingboxLabeled)
{
  if (!boundingboxLabeled.size() == 0)
  {
    std::string id = datatypeGroup + "/" + uuid;

    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    for (auto label : boundingboxLabeled)
    {
      labels.push_back(label->label()->str());
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

void GeneralIOFbs::writeBoundingBox2DLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeled>>& boundingbox2DLabeled)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!boundingbox2DLabeled.size() == 0)
  {
    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    for (auto label : boundingbox2DLabeled)
    {
      labels.push_back(label->label()->str());
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

void GeneralIOFbs::readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid,
                                            std::vector<std::string> labels,
                                            std::vector<std::vector<double>> boundingBoxes)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!m_file->exist(id + "/" + LABELBB))
  {
    std::cout << "id " << id + "/" + LABELBB << " does not exist in file " << m_file->getName() << std::endl;
    return;
  }
  if (!m_file->exist(id + "/" + LABELBBBOXES))
  {
    std::cout << "id " << id + "/" + LABELBBBOXES << " does not exist in file " << m_file->getName() << std::endl;
    return;
  }

  HighFive::DataSet datasetLabels = m_file->getDataSet(id + "/" + LABELBB);
  datasetLabels.read(labels);

  HighFive::DataSet datasetBoxes = m_file->getDataSet(id + "/" + LABELBBBOXES);
  datasetBoxes.read(boundingBoxes);

  if (labels.size() != boundingBoxes.size())
  {
    std::cout << "size of labels (" << labels.size() << ") and size of bounding boxes (" << boundingBoxes.size()
              << ") do not fit." << std::endl;
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

void GeneralIOFbs::writeLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid,
                                      const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>& labelsGeneral)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!labelsGeneral.size() == 0)
  {
    std::vector<std::string> labels;
    for (auto label : labelsGeneral)
    {
      labels.push_back(label->str());
    }

    HighFive::DataSet datasetLabels =
        m_file->createDataSet<std::string>(id + "/" + LABELGENERAL, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    m_file->flush();
  }
}

void GeneralIOFbs::readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid,
                                     std::vector<std::string> labels)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!m_file->exist(id + "/" + LABELGENERAL))
  {
    std::cout << "id " << id + "/" + LABELGENERAL << " does not exist in file " << m_file->getName() << std::endl;
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

void GeneralIOFbs::writeProjectname(const std::string& projectname)
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

std::string GeneralIOFbs::readProjectname()
{
  std::string projectname;
  if (m_file->hasAttribute(PROJECTNAME))
  {
    m_file->getAttribute(PROJECTNAME).read(projectname);
  }
  return projectname;
}

void GeneralIOFbs::writeProjectFrameId(const std::string& frameId)
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

std::string GeneralIOFbs::readProjectFrameId()
{
  std::string frameId;
  if (m_file->hasAttribute(PROJECTFRAMEID))
  {
    m_file->getAttribute(PROJECTFRAMEID).read(frameId);
  }
  return frameId;
}

} /* namespace seerep_hdf5 */
