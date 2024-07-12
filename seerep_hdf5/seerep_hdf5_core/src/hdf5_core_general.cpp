#include "seerep_hdf5_core/hdf5_core_general.h"

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
  writeAttributeToHdf5<std::string>(*m_file, PROJECTNAME, projectname);
  m_file->flush();
}

std::string Hdf5CoreGeneral::readProjectname()
{
  return readAttributeFromHdf5<std::string>(*m_file, PROJECTNAME, m_file->getName());
}

void Hdf5CoreGeneral::writeProjectFrameId(const std::string& frameId)
{
  writeFrameId(*m_file, PROJECTFRAMEID, frameId);
  m_file->flush();
}

std::string Hdf5CoreGeneral::readProjectFrameId()
{
  return readFrameId(*m_file, PROJECTFRAMEID, std::filesystem::path(m_file->getName()).filename().stem());
}

void Hdf5CoreGeneral::writeVersion(const std::string& version)
{
  writeAttributeToHdf5<std::string>(*m_file, VERSION, version);
  m_file->flush();
}

const std::optional<std::string> Hdf5CoreGeneral::readVersion()
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string version;
  try
  {
    version = readAttributeFromHdf5<std::string>(*m_file, VERSION, m_file->getName());
  }
  catch (const std::exception& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning) << e.what();
    return std::nullopt;
  }

  return version;
}

void Hdf5CoreGeneral::readLabelsAndAddToLabelsPerCategory(
    const std::string& datatypeGroup, const std::string& uuid,
    std::unordered_map<std::string, seerep_core_msgs::LabelDatumaro>& labelsCategoryMap)
{
  std::vector<std::string> labelCategories, datumaroJsonPerCategory;
  std::vector<std::vector<seerep_core_msgs::Label>> labelsPerCategory;
  readLabels(datatypeGroup, uuid, labelCategories, labelsPerCategory, datumaroJsonPerCategory);

  for (std::size_t i = 0; i < labelCategories.size(); i++)
  {
    labelsCategoryMap.emplace(labelCategories.at(i),
                              seerep_core_msgs::LabelDatumaro{ .labels = labelsPerCategory.at(i),
                                                               .datumaroJson = datumaroJsonPerCategory.at(i) }

    );
  }
}

void Hdf5CoreGeneral::readLabels(const std::string& datatypeGroup, const std::string& uuid,
                                 std::vector<std::string>& labelCategories,
                                 std::vector<std::vector<seerep_core_msgs::Label>>& labelsPerCategory,
                                 std::vector<std::string>& datumaroJsonPerCategory)
{
  boost::uuids::string_generator gen;

  const std::string id = datatypeGroup + "/" + uuid;
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "loading labels general of " << id;

  getLabelCategories(id, LABEL, labelCategories);

  for (const std::string& category : labelCategories)
  {
    std::vector<seerep_core_msgs::Label> label;
    std::vector<std::string> labels = readDataset<std::vector<std::string>>(id + "/" + LABEL + "_" + category);
    std::vector<int> labelsIdDatumaro = readDataset<std::vector<int>>(id + "/" + LABEL_ID_DATUMARO + "_" + category);
    std::vector<std::string> instances =
        readDataset<std::vector<std::string>>(id + "/" + LABELINSTANCES + "_" + category);
    std::vector<int> instancesIdDatumaro =
        readDataset<std::vector<int>>(id + "/" + LABELINSTANCES_ID_DATUMARO + "_" + category);
    datumaroJsonPerCategory.push_back(readDataset<std::string>(id + "/" + DATUMARO_JSON + "_" + category));

    if (labels.size() != labelsIdDatumaro.size())
    {
      const std::string errorMsg = "Unequal size of labels and datumaro label ids in category: " + category;
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << errorMsg;
      throw std::runtime_error(errorMsg);
    }

    bool addInstances = false;
    if (!instances.empty() && !instancesIdDatumaro.empty())
    {
      if (!hasEqualSize(labels, labelsIdDatumaro, instances, instancesIdDatumaro))
      {
        const std::string errorMsg = "unequal size of instances and labels in: " + category;
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << errorMsg;
        throw std::runtime_error(errorMsg);
      }
      addInstances = true;
    }

    for (size_t i = 0; i < labels.size(); i++)
    {
      seerep_core_msgs::Label core_label;
      core_label.label = labels.at(i);
      core_label.labelIdDatumaro = labelsIdDatumaro.at(i);

      if (addInstances)
      {
        try
        {
          core_label.instanceIdDatumaro = instancesIdDatumaro.at(i);
          core_label.uuidInstance = gen(instances.at(i));
        }
        catch (std::runtime_error&)
        {
          core_label.uuidInstance = boost::uuids::nil_uuid();
          BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
              << "Could not convert instance uuid string to uuid: " << instances.at(i);
        }
      }
      label.push_back(core_label);
    }
    labelsPerCategory.push_back(label);
  }
}

void Hdf5CoreGeneral::writeLabels(const std::string& datatypeGroup, const std::string& uuid,
                                  const std::vector<seerep_core_msgs::LabelCategory>& LabelCategory)
{
  std::string id = datatypeGroup + "/" + uuid;
  for (auto labels : LabelCategory)
  {
    HighFive::DataSet datasetLabels =
        m_file->createDataSet<std::string>(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABEL + "_" + labels.category,
                                           HighFive::DataSpace::From(labels.labels));
    datasetLabels.write(labels.labels);

    HighFive::DataSet datasetLabelsIdDatumaro = m_file->createDataSet<int>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABEL_ID_DATUMARO + "_" + labels.category,
        HighFive::DataSpace::From(labels.labelsIdDatumaro));
    datasetLabelsIdDatumaro.write(labels.labelsIdDatumaro);

    HighFive::DataSet datasetInstances = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELINSTANCES + "_" + labels.category,
        HighFive::DataSpace::From(labels.instances));
    datasetInstances.write(labels.instances);

    HighFive::DataSet datasetInstancesIdDatumaro = m_file->createDataSet<int>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELINSTANCES_ID_DATUMARO + "_" + labels.category,
        HighFive::DataSpace::From(labels.instancesIdDatumaro));
    datasetInstancesIdDatumaro.write(labels.instancesIdDatumaro);

    HighFive::DataSet datasetDatumaroJson = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::DATUMARO_JSON + "_" + labels.category,
        HighFive::DataSpace::From(labels.datumaroJson));
    datasetDatumaroJson.write(labels.datumaroJson);
  }

  m_file->flush();
}

const std::string Hdf5CoreGeneral::tf2_frame_id(const std::string& frame_id) const
{
  /* leading slahes are not allowed with tf2 */
  if (!frame_id.empty() && frame_id.at(0) == '/')
  {
    return frame_id.substr(1);
  }
  return frame_id;
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
  {
    group.createAttribute(AABB_FIELD, aabbPoints);
  }
  else
  {
    group.getAttribute(AABB_FIELD).write(aabbPoints);
  }

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

void Hdf5CoreGeneral::checkExists(const std::string& id)
{
  if (!m_file->exist(id))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "id " << id << " does not exist in file " << m_file->getName();
    throw std::invalid_argument("id " + id + " does not exist in file " + m_file->getName());
  }
}

bool Hdf5CoreGeneral::exists(const std::string& path) const
{
  if (!m_file->exist(path))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "HDF5 path: " << path << " does not exist in file: " << m_file->getName();
    return false;
  }
  return true;
}

std::shared_ptr<HighFive::Group> Hdf5CoreGeneral::getHdf5Group(const std::string& group_path, bool create)
{
  if (exists(group_path))
  {
    return std::make_shared<HighFive::Group>(m_file->getGroup(group_path));
  }
  else if (create)
  {
    return std::make_shared<HighFive::Group>(m_file->createGroup(group_path));
  }
  return nullptr;
}

void Hdf5CoreGeneral::writeGeodeticLocation(const seerep_core_msgs::GeodeticCoordinates& geo_coordinates)
{
  writeAttributeToHdf5<std::string>(*m_file, GEODETICLOCATION_COORDINATESYSTEM, geo_coordinates.coordinateSystem);
  writeAttributeToHdf5<double>(*m_file, GEODETICLOCATION_ALTITUDE, geo_coordinates.altitude);
  writeAttributeToHdf5<double>(*m_file, GEODETICLOCATION_LATITUDE, geo_coordinates.latitude);
  writeAttributeToHdf5<double>(*m_file, GEODETICLOCATION_LONGITUDE, geo_coordinates.longitude);
  m_file->flush();
}

std::optional<seerep_core_msgs::GeodeticCoordinates> Hdf5CoreGeneral::readGeodeticLocation()
{
  seerep_core_msgs::GeodeticCoordinates geocoords;
  try
  {
    geocoords.coordinateSystem =
        readAttributeFromHdf5<std::string>(*m_file, GEODETICLOCATION_COORDINATESYSTEM, m_file->getName());
    geocoords.altitude = readAttributeFromHdf5<double>(*m_file, GEODETICLOCATION_ALTITUDE, m_file->getName());
    geocoords.latitude = readAttributeFromHdf5<double>(*m_file, GEODETICLOCATION_LATITUDE, m_file->getName());
    geocoords.longitude = readAttributeFromHdf5<double>(*m_file, GEODETICLOCATION_LONGITUDE, m_file->getName());
  }
  catch (const std::invalid_argument& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "Geographic coordinates of the project " << m_file->getName() << " could not be read!";
    return std::nullopt;
  }
  return geocoords;
}

std::shared_ptr<HighFive::DataSet> Hdf5CoreGeneral::getHdf5DataSet(const std::string& hdf5DataSetPath)
{
  if (exists(hdf5DataSetPath))
  {
    return std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DataSetPath));
  }
  return nullptr;
}

void Hdf5CoreGeneral::getLabelCategories(std::string id, std::string labelType,
                                         std::vector<std::string>& matchingLabelCategory)
{
  std::vector<std::string> leafObjects = m_file->getGroup(id).listObjectNames();

  std::string searchString = labelType + "_";

  for (auto obj : leafObjects)
  {
    // check if obj name starts with labelType
    if (obj.rfind(searchString, 0) == 0)
    {
      // get category from postfix
      matchingLabelCategory.push_back(obj.substr(searchString.size()));
    }
  }
}

}  // namespace seerep_hdf5_core
