#include "seerep_core/core_project.h"

namespace seerep_core
{
CoreProject::CoreProject(const boost::uuids::uuid& uuid, const std::string path) : m_uuid(uuid), m_path(path)
{
  createHdf5Io(m_path);

  m_projectname = m_ioGeneral->readProjectname();
  m_frameId = m_ioGeneral->readProjectFrameId();
  /// TODO use the advantages of std::optional
  auto geodeticCoordinates = m_ioGeneral->readGeodeticLocation();

  if (geodeticCoordinates)
  {
    m_geodeticCoordinates = geodeticCoordinates.value();
  }

  recreateDatatypes();
}
CoreProject::CoreProject(const boost::uuids::uuid& uuid, const std::string path, const std::string projectname,
                         const std::string mapFrameId, const seerep_core_msgs::GeodeticCoordinates geodeticCoords)
  : m_uuid(uuid), m_path(path), m_projectname(projectname), m_frameId(mapFrameId), m_geodeticCoordinates(geodeticCoords)
{
  createHdf5Io(m_path);
  m_ioGeneral->writeProjectname(m_projectname);
  m_ioGeneral->writeProjectFrameId(m_frameId);
  m_ioGeneral->writeGeodeticLocation(m_geodeticCoordinates.value());
  recreateDatatypes();
}
CoreProject::~CoreProject()
{
}

std::string CoreProject::getName()
{
  return m_projectname;
}
std::string CoreProject::getFrameId()
{
  return m_frameId;
}

seerep_core_msgs::QueryResultProject CoreProject::getDataset(const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResultProject result;
  result.projectUuid = m_uuid;
  result.dataOrInstanceUuids = m_coreDatasets->getData(m_coreTfs->transformQuery(query, m_frameId));
  return result;
}

seerep_core_msgs::QueryResultProject CoreProject::getInstances(const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResultProject result;
  result.projectUuid = m_uuid;
  result.dataOrInstanceUuids = m_coreDatasets->getInstances(m_coreTfs->transformQuery(query, m_frameId));

  return result;
}

seerep_core_msgs::GeodeticCoordinates CoreProject::getGeodeticCoordinates()
{
  if (m_geodeticCoordinates)
  {
    return m_geodeticCoordinates.value();
  }
  else
  {
    return seerep_core_msgs::GeodeticCoordinates();
  }
}

void CoreProject::addDataset(const seerep_core_msgs::DatasetIndexable& dataset)
{
  m_coreDatasets->addDataset(dataset);
}

void CoreProject::addLabels(const seerep_core_msgs::Datatype& datatype,
                            const std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>>&
                                labelWithInstancePerCategory,
                            const boost::uuids::uuid& msgUuid)
{
  m_coreDatasets->addLabels(datatype, labelWithInstancePerCategory, msgUuid);
}

void CoreProject::addTF(const geometry_msgs::TransformStamped& tf)
{
  m_coreTfs->addDataset(tf);
}
std::optional<geometry_msgs::TransformStamped> CoreProject::getTF(const seerep_core_msgs::QueryTf& transformQuery)
{
  return m_coreTfs->getData(transformQuery.timestamp.seconds, transformQuery.timestamp.nanos,
                            transformQuery.parentFrameId, transformQuery.childFrameId);
}
std::vector<std::string> CoreProject::getFrames()
{
  return m_coreTfs->getFrames();
}

void CoreProject::addCameraIntrinsics(const seerep_core_msgs::camera_intrinsics& ci)
{
  m_coreCameraIntrinsics->addData(ci);
}

std::optional<seerep_core_msgs::camera_intrinsics>
CoreProject::getCameraIntrinsics(const seerep_core_msgs::camera_intrinsics_query& ci_query)
{
  return m_coreCameraIntrinsics->getData(ci_query);
}

std::shared_ptr<std::mutex> CoreProject::getHdf5FileMutex()
{
  return m_write_mtx;
}
std::shared_ptr<HighFive::File> CoreProject::getHdf5File()
{
  return m_hdf5_file;
}

void CoreProject::createHdf5Io(std::string path)
{
  m_write_mtx = std::make_shared<std::mutex>();
  m_hdf5_file = std::make_shared<HighFive::File>(path, HighFive::File::ReadWrite | HighFive::File::Create);

  m_ioGeneral = std::make_shared<seerep_hdf5_core::Hdf5CoreGeneral>(m_hdf5_file, m_write_mtx);
  m_ioTf = std::make_shared<seerep_hdf5_core::Hdf5CoreTf>(m_hdf5_file, m_write_mtx);
  m_ioCI = std::make_shared<seerep_hdf5_core::Hdf5CoreCameraIntrinsics>(m_hdf5_file, m_write_mtx);
  m_ioInstance = std::make_shared<seerep_hdf5_core::Hdf5CoreInstance>(m_hdf5_file, m_write_mtx);

  m_ioPointCloud = std::make_shared<seerep_hdf5_core::Hdf5CorePointCloud>(m_hdf5_file, m_write_mtx);
  m_ioPoint = std::make_shared<seerep_hdf5_core::Hdf5CorePoint>(m_hdf5_file, m_write_mtx);
  m_ioImage = std::make_shared<seerep_hdf5_core::Hdf5CoreImage>(m_hdf5_file, m_write_mtx);
}
void CoreProject::recreateDatatypes()
{
  m_coreTfs = std::make_shared<seerep_core::CoreTf>(m_ioTf);
  m_coreCameraIntrinsics = std::make_shared<seerep_core::CoreCameraIntrinsics>(m_ioCI);
  m_coreInstances = std::make_shared<seerep_core::CoreInstances>(m_ioInstance);
  m_coreDatasets = std::make_unique<seerep_core::CoreDataset>(m_coreTfs, m_coreInstances, m_frameId);

  m_coreDatasets->addDatatype(seerep_core_msgs::Datatype::Image, m_ioImage);
  m_coreDatasets->addDatatype(seerep_core_msgs::Datatype::PointCloud, m_ioPointCloud);
  m_coreDatasets->addDatatype(seerep_core_msgs::Datatype::Point, m_ioPoint);

  std::vector<std::string> datatypeNames = m_ioGeneral->getGroupDatasets("");
  for (auto datatypeName : datatypeNames)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "found datatype " << datatypeName << " in HDF5 file.";
  }
}

} /* namespace seerep_core */
