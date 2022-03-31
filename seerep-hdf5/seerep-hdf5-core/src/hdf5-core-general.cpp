#include "seerep-hdf5-core/hdf5-core-general.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_core
{
Hdf5CoreGeneral::Hdf5CoreGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : m_file(file), m_write_mtx(write_mtx)
{
}

// std::optional<seerep_core_msgs::DatasetIndexable> Hdf5CoreGeneral::readDataForIndices(const std::string& datatypeGroup,
//                                                                                       const std::string& uuid)
// {
//   std::string id = datatypeGroup + "/" + uuid;
//   HighFive::Group group = m_file->getGroup(id);

//   seerep_core_msgs::DatasetIndexable data;

//   group.getAttribute(HEADER_FRAME_ID).read(data.header.frameId);

//   group.getAttribute(HEADER_STAMP_SECONDS).read(data.header.timestamp.seconds);
//   group.getAttribute(HEADER_STAMP_NANOS).read(data.header.timestamp.nanos);

//   readAABB(datatypeGroup, uuid, data.boundingbox);
//   std::vector<std::string> labelsGeneral;
//   readLabelsGeneral(datatypeGroup, uuid, labelsGeneral);
//   std::vector<std::string> labelsBB;
//   std::vector<std::vector<double>> dummy;
//   readBoundingBox2DLabeled(datatypeGroup, uuid, labelsBB, dummy);

//   data.labels.insert(std::end(data.labels), std::begin(labelsGeneral), std::end(labelsGeneral));
//   data.labels.insert(std::end(data.labels), std::begin(labelsBB), std::end(labelsBB));

//   return data;
// }

// std::optional<std::string> Hdf5CoreGeneral::readFrameId(const std::string& datatypeGroup, const std::string& uuid)
// {
//   std::string id = datatypeGroup + "/" + uuid;
//   std::string hdf5DatasetRawDataPath = id + "/" + RAWDATA;
//   if (!m_file->exist(hdf5DatasetRawDataPath))
//   {
//     std::cout << "id " << hdf5DatasetRawDataPath << " does not exist in file " << m_file->getName() << std::endl;
//     throw std::invalid_argument("id " + hdf5DatasetRawDataPath + " does not exist in file " + m_file->getName());
//   }
//   std::cout << "get dataset " << hdf5DatasetRawDataPath << std::endl;
//   std::shared_ptr<HighFive::DataSet> data_set_ptr =
//       std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRawDataPath));

//   if (data_set_ptr->hasAttribute(HEADER_FRAME_ID))
//   {
//     std::string frameId;
//     data_set_ptr->getAttribute(HEADER_FRAME_ID).read(frameId);
//     return frameId;
//   }
//   else
//   {
//     return std::nullopt;
//   }
// }

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

// void Hdf5CoreGeneral::deleteAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField)
// {
//   if (dataSetPtr->hasAttribute(attributeField))
//   {
//     dataSetPtr->deleteAttribute(attributeField);
//     m_file->flush();
//   }
// }

// void Hdf5CoreGeneral::writeAABB(
//     const std::string& datatypeGroup, const std::string& uuid,
//     const boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb)
// {
//   std::string id = datatypeGroup + "/" + uuid;
//   if (!m_file->exist(id))
//   {
//     std::cout << "id " << id << " does not exist in file " << m_file->getName() << std::endl;
//     throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
//   }
//   std::cout << "get group " << id << std::endl;
//   HighFive::Group group = m_file->getGroup(id);

//   std::vector<float> aabbPoints{ aabb.min_corner().get<0>(), aabb.min_corner().get<1>(), aabb.min_corner().get<2>(),
//                                  aabb.max_corner().get<0>(), aabb.max_corner().get<1>(), aabb.max_corner().get<2>() };

//   std::cout << "write AABB as attribute" << std::endl;
//   if (!group.hasAttribute(AABB_FIELD))
//     group.createAttribute(AABB_FIELD, aabbPoints);
//   else
//     group.getAttribute(AABB_FIELD).write(aabbPoints);

//   m_file->flush();
// }

// void Hdf5CoreGeneral::readAABB(const std::string& datatypeGroup, const std::string& uuid, seerep_core_msgs::AABB& aabb)
// {
//   std::string id = datatypeGroup + "/" + uuid;

//   if (!m_file->exist(id))
//   {
//     std::cout << "id " << id << " does not exist in file " << m_file->getName() << std::endl;
//     throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
//   }
//   std::cout << "get group " << id << std::endl;
//   HighFive::Group group = m_file->getGroup(id);
//   if (group.hasAttribute(AABB_FIELD))
//   {
//     std::vector<float> aabbPoints;
//     group.getAttribute(AABB_FIELD).read(aabbPoints);

//     aabb.min_corner().set<0>(aabbPoints.at(0));
//     aabb.min_corner().set<1>(aabbPoints.at(1));
//     aabb.min_corner().set<2>(aabbPoints.at(2));
//     aabb.max_corner().set<0>(aabbPoints.at(3));
//     aabb.max_corner().set<1>(aabbPoints.at(4));
//     aabb.max_corner().set<2>(aabbPoints.at(5));
//   }
// }

// bool Hdf5CoreGeneral::hasAABB(const std::string& datatypeGroup, const std::string& uuid)
// {
//   std::string id = datatypeGroup + "/" + uuid;

//   if (!m_file->exist(id))
//   {
//     std::cout << "id " << id << " does not exist in file " << m_file->getName() << std::endl;
//     throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
//   }
//   std::cout << "get group " << id << std::endl;
//   HighFive::Group group = m_file->getGroup(id);
//   return group.hasAttribute(AABB_FIELD);
// }

// void Hdf5CoreGeneral::readTimeFromRaw(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs,
//                                       int64_t& nanos)
// {
//   readTime(datatypeGroup, uuid + "/" + RAWDATA, secs, nanos);
// }

// void Hdf5CoreGeneral::readTime(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs, int64_t& nanos)
// {
//   std::string id = datatypeGroup + "/" + uuid;

//   if (!m_file->exist(id))
//   {
//     std::cout << "id " << id << " does not exist in file " << m_file->getName() << std::endl;
//     throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
//   }

//   switch (m_file->getObjectType(id))
//   {
//     case HighFive::ObjectType::Group:
//     {
//       std::cout << "get group " << id << std::endl;
//       HighFive::Group group = m_file->getGroup(id);
//       if (group.hasAttribute(HEADER_STAMP_SECONDS))
//       {
//         group.getAttribute(HEADER_STAMP_SECONDS).read(secs);
//       }
//       else
//       {
//         throw std::invalid_argument("id " + id + " has no attribute " + HEADER_STAMP_SECONDS);
//       }
//       if (group.hasAttribute(HEADER_STAMP_NANOS))
//       {
//         group.getAttribute(HEADER_STAMP_NANOS).read(nanos);
//       }
//       else
//       {
//         throw std::invalid_argument("id " + id + " has no attribute " + HEADER_STAMP_NANOS);
//       }
//     };
//     break;

//     case HighFive::ObjectType::Dataset:
//     {
//       std::cout << "get group " << id << std::endl;
//       HighFive::DataSet dataset = m_file->getDataSet(id);
//       if (dataset.hasAttribute(HEADER_STAMP_SECONDS))
//       {
//         dataset.getAttribute(HEADER_STAMP_SECONDS).read(secs);
//       }
//       else
//       {
//         throw std::invalid_argument("id " + id + " has no attribute " + HEADER_STAMP_SECONDS);
//       }
//       if (dataset.hasAttribute(HEADER_STAMP_NANOS))
//       {
//         dataset.getAttribute(HEADER_STAMP_NANOS).read(nanos);
//       }
//       else
//       {
//         throw std::invalid_argument("id " + id + " has no attribute " + HEADER_STAMP_NANOS);
//       }
//     };
//     break;

//     default:
//       secs = std::numeric_limits<uint64_t>::min();
//       nanos = std::numeric_limits<uint64_t>::min();
//   }
// }

// void Hdf5CoreGeneral::writeTimeToRaw(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
//                                      const int64_t& nanos)
// {
//   writeTime(datatypeGroup, uuid + "/" + RAWDATA, secs, nanos);
// }

// void Hdf5CoreGeneral::writeTime(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
//                                 const int64_t& nanos)
// {
//   std::string id = datatypeGroup + "/" + uuid;

//   if (!m_file->exist(id))
//   {
//     std::cout << "id " << id << " does not exist in file " << m_file->getName() << std::endl;
//     throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
//   }

//   switch (m_file->getObjectType(id))
//   {
//     case HighFive::ObjectType::Group:
//     {
//       std::cout << "get group " << id << std::endl;
//       HighFive::Group group = m_file->getGroup(id);
//       if (group.hasAttribute(HEADER_STAMP_SECONDS))
//       {
//         group.getAttribute(HEADER_STAMP_SECONDS).write(secs);
//       }
//       else
//       {
//         group.createAttribute(HEADER_STAMP_SECONDS, secs);
//       }
//       if (group.hasAttribute(HEADER_STAMP_NANOS))
//       {
//         group.getAttribute(HEADER_STAMP_NANOS).write(nanos);
//       }
//       else
//       {
//         group.createAttribute(HEADER_STAMP_NANOS, nanos);
//       }
//       m_file->flush();
//       return;
//     };

//     case HighFive::ObjectType::Dataset:
//     {
//       std::cout << "get group " << id << std::endl;
//       HighFive::DataSet dataset = m_file->getDataSet(id);
//       if (dataset.hasAttribute(HEADER_STAMP_SECONDS))
//       {
//         dataset.getAttribute(HEADER_STAMP_SECONDS).write(secs);
//       }
//       else
//       {
//         dataset.createAttribute(HEADER_STAMP_SECONDS, secs);
//       }
//       if (dataset.hasAttribute(HEADER_STAMP_NANOS))
//       {
//         dataset.getAttribute(HEADER_STAMP_NANOS).write(nanos);
//       }
//       else
//       {
//         dataset.createAttribute(HEADER_STAMP_NANOS, nanos);
//       }
//       m_file->flush();
//       return;
//     };
//     default:
//       return;
//   }
// }

// bool Hdf5CoreGeneral::hasTimeRaw(const std::string& datatypeGroup, const std::string& uuid)
// {
//   return hasTime(datatypeGroup, uuid + "/" + RAWDATA);
// }

// bool Hdf5CoreGeneral::hasTime(const std::string& datatypeGroup, const std::string& uuid)
// {
//   std::string id = datatypeGroup + "/" + uuid;
//   if (!m_file->exist(id))
//   {
//     std::cout << "id " << id << " does not exist in file " << m_file->getName() << std::endl;
//     throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
//   }

//   switch (m_file->getObjectType(id))
//   {
//     case HighFive::ObjectType::Group:
//       std::cout << "get group " << id << std::endl;
//       return m_file->getGroup(id).hasAttribute(HEADER_STAMP_SECONDS) &&
//              m_file->getGroup(id).hasAttribute(HEADER_STAMP_NANOS);

//     case HighFive::ObjectType::Dataset:
//       std::cout << "get dataset " << id << std::endl;
//       return m_file->getDataSet(id).hasAttribute(HEADER_STAMP_SECONDS) &&
//              m_file->getDataSet(id).hasAttribute(HEADER_STAMP_NANOS);

//     default:
//       return false;
//   }
// }

// void Hdf5CoreGeneral::readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid,
//                                                std::vector<std::string> labels,
//                                                std::vector<std::vector<double>> boundingBoxes)
// {
//   std::string id = datatypeGroup + "/" + uuid;
//   if (!m_file->exist(id + "/" + LABELBB))
//   {
//     std::cout << "id " << id + "/" + LABELBB << " does not exist in file " << m_file->getName() << std::endl;
//     return;
//   }
//   if (!m_file->exist(id + "/" + LABELBBBOXES))
//   {
//     std::cout << "id " << id + "/" + LABELBBBOXES << " does not exist in file " << m_file->getName() << std::endl;
//     return;
//   }

//   HighFive::DataSet datasetLabels = m_file->getDataSet(id + "/" + LABELBB);
//   datasetLabels.read(labels);

//   HighFive::DataSet datasetBoxes = m_file->getDataSet(id + "/" + LABELBBBOXES);
//   datasetBoxes.read(boundingBoxes);

//   if (labels.size() != boundingBoxes.size())
//   {
//     std::cout << "size of labels (" << labels.size() << ") and size of bounding boxes (" << boundingBoxes.size()
//               << ") do not fit." << std::endl;
//     labels.clear();
//     boundingBoxes.clear();
//     return;
//   }
// }

// void Hdf5CoreGeneral::readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid,
//                                         std::vector<std::string> labels)
// {
//   std::string id = datatypeGroup + "/" + uuid;
//   if (!m_file->exist(id + "/" + LABELGENERAL))
//   {
//     std::cout << "id " << id + "/" + LABELGENERAL << " does not exist in file " << m_file->getName() << std::endl;
//     return;
//   }

//   HighFive::DataSet datasetLabels = m_file->getDataSet(id + "/" + LABELGENERAL);
//   datasetLabels.read(labels);
// }

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

}  // namespace seerep_hdf5_core
