#include "seerep-hdf5-pb/hdf5-pb-general.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_pb
{
Hdf5PbGeneral::Hdf5PbGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : m_file(file), m_write_mtx(write_mtx)
{
}

std::optional<std::string> Hdf5PbGeneral::readFrameId(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  std::string hdf5DatasetRawDataPath = id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::RAWDATA;

  checkExists(hdf5DatasetRawDataPath);

  std::cout << "get dataset " << hdf5DatasetRawDataPath << std::endl;
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

std::vector<std::string> Hdf5PbGeneral::getGroupDatasets(const std::string& id)
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

void Hdf5PbGeneral::checkExists(const std::string& id)
{
  if (!m_file->exist(id))
  {
    std::cout << "id " << id << " does not exist in file " << m_file->getName() << std::endl;
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }
}

void Hdf5PbGeneral::deleteAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField)
{
  if (dataSetPtr->hasAttribute(attributeField))
  {
    dataSetPtr->deleteAttribute(attributeField);
    m_file->flush();
  }
}

void Hdf5PbGeneral::writeAABB(
    const std::string& datatypeGroup, const std::string& uuid,
    const boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  std::cout << "get group " << id << std::endl;
  HighFive::Group group = m_file->getGroup(id);

  std::vector<float> aabbPoints{ aabb.min_corner().get<0>(), aabb.min_corner().get<1>(), aabb.min_corner().get<2>(),
                                 aabb.max_corner().get<0>(), aabb.max_corner().get<1>(), aabb.max_corner().get<2>() };

  std::cout << "write AABB as attribute" << std::endl;
  if (!group.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::AABB_FIELD))
    group.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::AABB_FIELD, aabbPoints);
  else
    group.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::AABB_FIELD).write(aabbPoints);

  m_file->flush();
}

void Hdf5PbGeneral::readAABB(
    const std::string& datatypeGroup, const std::string& uuid,
    boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  std::cout << "get group " << id << std::endl;
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

bool Hdf5PbGeneral::hasAABB(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  std::cout << "get group " << id << std::endl;
  HighFive::Group group = m_file->getGroup(id);
  return group.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::AABB_FIELD);
}

void Hdf5PbGeneral::readTimeFromRaw(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs,
                                    int64_t& nanos)
{
  readTime(datatypeGroup, uuid + "/" + seerep_hdf5_core::Hdf5CoreGeneral::RAWDATA, secs, nanos);
}

void Hdf5PbGeneral::readTime(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs, int64_t& nanos)
{
  std::string id = datatypeGroup + "/" + uuid;

  checkExists(id);

  switch (m_file->getObjectType(id))
  {
    case HighFive::ObjectType::Group:
    {
      std::cout << "get group " << id << std::endl;
      HighFive::Group group = m_file->getGroup(id);
      if (group.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS))
      {
        group.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS).read(secs);
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " +
                                    seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS);
      }
      if (group.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS))
      {
        group.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS).read(nanos);
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " +
                                    seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS);
      }
    };
    break;

    case HighFive::ObjectType::Dataset:
    {
      std::cout << "get group " << id << std::endl;
      HighFive::DataSet dataset = m_file->getDataSet(id);
      if (dataset.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS))
      {
        dataset.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS).read(secs);
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " +
                                    seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS);
      }
      if (dataset.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS))
      {
        dataset.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS).read(nanos);
      }
      else
      {
        throw std::invalid_argument("id " + id + " has no attribute " +
                                    seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS);
      }
    };
    break;

    default:
      secs = std::numeric_limits<uint64_t>::min();
      nanos = std::numeric_limits<uint64_t>::min();
  }
}

void Hdf5PbGeneral::writeTimeToRaw(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
                                   const int64_t& nanos)
{
  writeTime(datatypeGroup, uuid + "/" + seerep_hdf5_core::Hdf5CoreGeneral::RAWDATA, secs, nanos);
}

void Hdf5PbGeneral::writeTime(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
                              const int64_t& nanos)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  switch (m_file->getObjectType(id))
  {
    case HighFive::ObjectType::Group:
    {
      std::cout << "get group " << id << std::endl;
      HighFive::Group group = m_file->getGroup(id);
      if (group.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS))
      {
        group.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS).write(secs);
      }
      else
      {
        group.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS, secs);
      }
      if (group.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS))
      {
        group.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS).write(nanos);
      }
      else
      {
        group.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS, nanos);
      }
      m_file->flush();
      return;
    };

    case HighFive::ObjectType::Dataset:
    {
      std::cout << "get group " << id << std::endl;
      HighFive::DataSet dataset = m_file->getDataSet(id);
      if (dataset.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS))
      {
        dataset.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS).write(secs);
      }
      else
      {
        dataset.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS, secs);
      }
      if (dataset.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS))
      {
        dataset.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS).write(nanos);
      }
      else
      {
        dataset.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS, nanos);
      }
      m_file->flush();
      return;
    };
    default:
      return;
  }
}

bool Hdf5PbGeneral::hasTimeRaw(const std::string& datatypeGroup, const std::string& uuid)
{
  return hasTime(datatypeGroup, uuid + "/" + seerep_hdf5_core::Hdf5CoreGeneral::RAWDATA);
}

bool Hdf5PbGeneral::hasTime(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  checkExists(id);

  switch (m_file->getObjectType(id))
  {
    case HighFive::ObjectType::Group:
      std::cout << "get group " << id << std::endl;
      return m_file->getGroup(id).hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS) &&
             m_file->getGroup(id).hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS);

    case HighFive::ObjectType::Dataset:
      std::cout << "get dataset " << id << std::endl;
      return m_file->getDataSet(id).hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS) &&
             m_file->getDataSet(id).hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS);

    default:
      return false;
  }
}

void Hdf5PbGeneral::writeBoundingBoxLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const google::protobuf::RepeatedPtrField<::seerep::BoundingBoxLabeled>& boundingboxLabeled)
{
  if (!boundingboxLabeled.empty())
  {
    std::string id = datatypeGroup + "/" + uuid;

    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    std::vector<std::string> instances;
    for (auto label : boundingboxLabeled)
    {
      labels.push_back(label.labelwithinstance().label());
      std::vector<double> box{ label.boundingbox().point_min().x(), label.boundingbox().point_min().y(),
                               label.boundingbox().point_min().z(), label.boundingbox().point_max().x(),
                               label.boundingbox().point_max().y(), label.boundingbox().point_max().z() };
      boundingBoxes.push_back(box);
      instances.push_back(label.labelwithinstance().instanceuuid());
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

void Hdf5PbGeneral::writeBoundingBox2DLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const google::protobuf::RepeatedPtrField<::seerep::BoundingBox2DLabeled>& boundingbox2DLabeled)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!boundingbox2DLabeled.empty())
  {
    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    std::vector<std::string> instances;
    for (auto label : boundingbox2DLabeled)
    {
      labels.push_back(label.labelwithinstance().label());
      std::vector<double> box{ label.boundingbox().point_min().x(), label.boundingbox().point_min().y(),
                               label.boundingbox().point_max().x(), label.boundingbox().point_max().y() };
      boundingBoxes.push_back(box);

      instances.push_back(label.labelwithinstance().instanceuuid());
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

std::optional<google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeled>>
Hdf5PbGeneral::readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid)
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
    return std::nullopt;
  }

  std::vector<std::string> labels;
  std::vector<std::vector<double>> boundingBoxes;
  std::vector<std::string> instances;

  HighFive::DataSet datasetLabels = m_file->getDataSet(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB);
  datasetLabels.read(labels);

  HighFive::DataSet datasetBoxes = m_file->getDataSet(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES);
  datasetBoxes.read(boundingBoxes);

  HighFive::DataSet datasetInstances =
      m_file->getDataSet(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES);
  datasetInstances.read(instances);

  if (labels.size() != boundingBoxes.size() || labels.size() != instances.size())
  {
    std::cout << "size of labels (" << labels.size() << "), size of bounding boxes (" << boundingBoxes.size()
              << ") and size of instances (" << instances.size() << ") do not fit." << std::endl;
    return std::nullopt;
  }

  google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeled> result;

  for (long unsigned int i = 0; i < labels.size(); i++)
  {
    seerep::BoundingBox2DLabeled bblabeled;
    bblabeled.mutable_labelwithinstance()->set_label(labels.at(i));
    bblabeled.mutable_labelwithinstance()->set_instanceuuid(instances.at(i));

    bblabeled.mutable_boundingbox()->mutable_point_min()->set_x(boundingBoxes.at(i).at(0));
    bblabeled.mutable_boundingbox()->mutable_point_min()->set_y(boundingBoxes.at(i).at(1));
    bblabeled.mutable_boundingbox()->mutable_point_max()->set_x(boundingBoxes.at(i).at(2));
    bblabeled.mutable_boundingbox()->mutable_point_max()->set_y(boundingBoxes.at(i).at(3));

    result.Add(std::move(bblabeled));
  }

  return result;
}

void Hdf5PbGeneral::writeLabelsGeneral(
    const std::string& datatypeGroup, const std::string& uuid,
    const google::protobuf::RepeatedPtrField<seerep::LabelWithInstance>& labelsGeneralWithInstances)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!labelsGeneralWithInstances.empty())
  {
    std::vector<std::string> labels;
    std::vector<std::string> instances;
    for (auto labelWithInstances : labelsGeneralWithInstances)
    {
      labels.push_back(labelWithInstances.label());

      instances.push_back(labelWithInstances.instanceuuid());
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

std::optional<google::protobuf::RepeatedPtrField<seerep::LabelWithInstance>>
Hdf5PbGeneral::readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  try
  {
    checkExists(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL);
    checkExists(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES);
  }
  catch (std::invalid_argument const& e)
  {
    return std::nullopt;
  }

  std::vector<std::string> labels;
  std::vector<std::string> instances;

  HighFive::DataSet datasetLabels = m_file->getDataSet(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL);
  datasetLabels.read(labels);

  HighFive::DataSet datasetInstances =
      m_file->getDataSet(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES);
  datasetInstances.read(instances);

  if (labels.size() != instances.size())
  {
    std::cout << "size of labels (" << labels.size() << ") and size of instances (" << instances.size()
              << ") do not fit." << std::endl;
    return std::nullopt;
  }

  google::protobuf::RepeatedPtrField<seerep::LabelWithInstance> result;

  for (long unsigned int i = 0; i < labels.size(); i++)
  {
    seerep::LabelWithInstance labelWithInstance;
    labelWithInstance.set_label(labels.at(i));
    labelWithInstance.set_instanceuuid(instances.at(i));
    result.Add(std::move(labelWithInstance));
  }

  return result;
}

void Hdf5PbGeneral::writeProjectname(const std::string& projectname)
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

std::string Hdf5PbGeneral::readProjectname()
{
  std::string projectname;
  if (m_file->hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::PROJECTNAME))
  {
    m_file->getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::PROJECTNAME).read(projectname);
  }
  return projectname;
}

void Hdf5PbGeneral::writeProjectFrameId(const std::string& frameId)
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

std::string Hdf5PbGeneral::readProjectFrameId()
{
  std::string frameId;
  if (m_file->hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::PROJECTFRAMEID))
  {
    m_file->getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::PROJECTFRAMEID).read(frameId);
  }
  return frameId;
}

} /* namespace seerep_hdf5_pb */
