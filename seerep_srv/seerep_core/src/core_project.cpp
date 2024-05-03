#include "seerep_core/core_project.h"

namespace seerep_core
{
CoreProject::CoreProject(const boost::uuids::uuid& uuid, const std::string path) : m_uuid(uuid), m_path(path)
{
  createHdf5Io(m_path);

  m_projectname = m_ioGeneral->readProjectname();
  m_frameId = m_ioGeneral->readProjectFrameId();

  /* get optional class members */
  auto geodeticCoordinates = m_ioGeneral->readGeodeticLocation();
  auto version = m_ioGeneral->readVersion();

  if (version)
  {
    m_version = version.value();
  }

  if (geodeticCoordinates)
  {
    m_geodeticCoordinates = geodeticCoordinates.value();
  }
  recreateDatatypes();
}

CoreProject::CoreProject(const boost::uuids::uuid& uuid, const std::string path, const std::string projectname,
                         const std::string mapFrameId, const seerep_core_msgs::GeodeticCoordinates geodeticCoords,
                         const std::string version)
  : m_uuid(uuid)
  , m_path(path)
  , m_projectname(projectname)
  , m_frameId(mapFrameId)
  , m_geodeticCoordinates(geodeticCoords)
  , m_version(version)
{
  createHdf5Io(m_path);
  m_ioGeneral->writeProjectname(m_projectname);
  m_ioGeneral->writeProjectFrameId(m_frameId);
  m_ioGeneral->writeGeodeticLocation(m_geodeticCoordinates.value());
  m_ioGeneral->writeVersion(m_version);
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
const std::string CoreProject::getVersion()
{
  return m_version;
}

seerep_core_msgs::Polygon2D CoreProject::transformToMapFrame(const seerep_core_msgs::Polygon2D polygon)
{
  seerep_core_msgs::GeodeticCoordinates g = m_geodeticCoordinates.value();

  // https://proj.org/en/9.2/operations/conversions/topocentric.html
  // the proj pipeline has two steps, convert geodesic to cartesian coordinates
  // then project to topocentric coordinates
  std::string proj_pipeline = "step +proj=cartesian +ellps=" + g.coordinateSystem + " \nstep +proj=topocentric +ellps" +
                              g.coordinateSystem + "+lon_0=" + std::to_string(g.longitude) +
                              "lat_0=" + std::to_string(g.latitude) + "h_0=" + std::to_string(g.altitude);

  // PJ_FWD: forward, PJ_IDENT: identity, PJ_INV: inverse
  PJ_DIRECTION direction = PJ_FWD;

  seerep_core_msgs::Polygon2D transformed_polygon;

  // perform an affine transform to transpose the query polygon to map frame
  // the first argument of 0 is the thread context, the second is the pipeline string created above
  auto to_topographic = proj_create(0, proj_pipeline.c_str());

  // we traverse the polygon and apply the transform
  for (seerep_core_msgs::Point2D p : polygon.vertices)
  {
    PJ_COORD c = proj_coord(p.get<0>(), p.get<1>(), 0, 0);
    PJ_COORD t_coord = proj_trans(to_topographic, direction, c);

    seerep_core_msgs::Point2D transformed_p;
    p.set<0>(t_coord.xy.x);
    p.set<1>(t_coord.xy.y);
    transformed_polygon.vertices.push_back(transformed_p);
  }

  transformed_polygon.z = polygon.z - g.altitude;
  transformed_polygon.height = polygon.height;

  return transformed_polygon;
}

seerep_core_msgs::QueryResultProject CoreProject::getDataset(seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResultProject result;
  result.projectUuid = m_uuid;

  // is the query not in map frame?
  if (query.polygon && !query.inMapFrame)
  {
    query.polygon.value() = transformToMapFrame(query.polygon.value());
  }

  result.dataOrInstanceUuids = m_coreDatasets->getData(query);
  return result;
}

seerep_core_msgs::QueryResultProject CoreProject::getInstances(const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResultProject result;
  result.projectUuid = m_uuid;
  result.dataOrInstanceUuids = m_coreDatasets->getInstances(query);

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

std::optional<seerep_core_msgs::camera_intrinsics> CoreProject::getCameraIntrinsics(boost::uuids::uuid camIntrinsicsUuid)
{
  return m_coreCameraIntrinsics->getData(camIntrinsicsUuid);
}

bool CoreProject::cameraIntrinsicExists(boost::uuids::uuid camIntrinsicsUuid)
{
  return m_coreCameraIntrinsics->cameraIntrinsicExists(camIntrinsicsUuid);
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
  m_ioInstance = std::make_shared<seerep_hdf5_core::Hdf5CoreInstance>(m_hdf5_file, m_write_mtx);

  m_ioPointCloud = std::make_shared<seerep_hdf5_core::Hdf5CorePointCloud>(m_hdf5_file, m_write_mtx);
  m_ioPoint = std::make_shared<seerep_hdf5_core::Hdf5CorePoint>(m_hdf5_file, m_write_mtx);
  m_ioImage = std::make_shared<seerep_hdf5_core::Hdf5CoreImage>(m_hdf5_file, m_write_mtx);
}
void CoreProject::recreateDatatypes()
{
  m_coreTfs = std::make_shared<seerep_core::CoreTf>(m_ioTf);
  m_coreCameraIntrinsics = std::make_shared<seerep_core::CoreCameraIntrinsics>(m_hdf5_file, m_write_mtx);
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

seerep_core_msgs::AabbTime CoreProject::getTimeBounds(std::vector<seerep_core_msgs::Datatype> datatypes)
{
  return m_coreDatasets->getTimeBounds(datatypes);
}

seerep_core_msgs::AABB CoreProject::getSpatialBounds(std::vector<seerep_core_msgs::Datatype> datatypes)
{
  return m_coreDatasets->getSpatialBounds(datatypes);
}

std::unordered_set<std::string> CoreProject::getAllCategories(std::vector<seerep_core_msgs::Datatype> datatypes)
{
  return m_coreDatasets->getAllCategories(datatypes);
}

std::unordered_set<std::string> CoreProject::getAllLabels(std::vector<seerep_core_msgs::Datatype> datatypes,
                                                          std::string category)
{
  return m_coreDatasets->getAllLabels(datatypes, category);
}

} /* namespace seerep_core */
