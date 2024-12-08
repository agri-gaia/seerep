#include "seerep_core/core_project.h"

namespace seerep_core
{
CoreProject::CoreProject(const boost::uuids::uuid& uuid, const std::string path)
  : m_uuid(uuid), m_path(path)
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

CoreProject::CoreProject(
    const boost::uuids::uuid& uuid, const std::string path,
    const std::string projectname, const std::string mapFrameId,
    const seerep_core_msgs::GeodeticCoordinates geodeticCoords,
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

void CoreProject::transformPointFwd(seerep_core_msgs::Point& p,
                                    PJ* proj_tf_rawptr)
{
  PJ_COORD coord = proj_coord(p.get<0>(), p.get<1>(), p.get<2>(), 0);

  // PJ_FWD: forward, PJ_IDENT: identity, PJ_INV: inverse
  PJ_COORD t_coord = proj_trans(proj_tf_rawptr, PJ_FWD, coord);

  p.set<0>(t_coord.xyz.x);
  p.set<1>(t_coord.xyz.y);
  p.set<2>(t_coord.xyz.z);
}

seerep_core_msgs::Polygon2D
CoreProject::transformToMapFrame(const seerep_core_msgs::Polygon2D& polygon,
                                 const std::string& query_crs)
{
  // get projects coordinates, those are also the origin for the map frame in
  // the global earth frame
  seerep_core_msgs::GeodeticCoordinates g = m_geodeticCoordinates.value();

  // check if a coordinate system on the project is not set
  if (g.crsString == "")
  {
    throw std::invalid_argument(
        "Could not transform to the projects map frame, "
        "coordinate system string for project is not set!");
  }

  PJ_CONTEXT* ctx = proj_context_create();

  std::vector<double> transformed_z_coords;
  std::vector<seerep_core_msgs::Point> transformed_points3d;

  seerep_core_msgs::Polygon2D transformed_polygon;
  // set values which will not  change
  transformed_polygon.height = polygon.height;

  // when the query was made in a seperate coordinate reference system
  if (query_crs != "project")
  {
    PJ* to_project_crs_rawptr = proj_create_crs_to_crs(
        ctx, query_crs.c_str(), g.crsString.c_str(), nullptr);

    // check whether an error occured
    if (to_project_crs_rawptr == nullptr)
    {
      int errnum = proj_context_errno(ctx);
      if (errnum == 0)
      {
        proj_context_destroy(ctx);
        throw std::runtime_error("An error occured during transform to the "
                                 "projects coordinate reference system, "
                                 "couldn't identify the error code!");
      }
      std::string err_info(proj_errno_string(errnum));
      proj_context_destroy(ctx);

      throw std::invalid_argument("Could not transform query polygon to the "
                                  "projects coordinate reference system: " +
                                  err_info);
    }

    for (auto&& p : polygon.vertices)
    {
      // as we get the polygon in lat, long and it must be passed as long, lat
      seerep_core_msgs::Point p3d{ p.get<0>(), p.get<1>(),
                                   static_cast<float>(polygon.z) };

      // transformed point
      transformPointFwd(p3d, to_project_crs_rawptr);
      transformed_points3d.push_back(p3d);
    }

    proj_destroy(to_project_crs_rawptr);
  }

  PJ* crs_rawptr = proj_create(ctx, g.crsString.c_str());

  if (crs_rawptr == nullptr)
  {
    int errnum = proj_context_errno(ctx);
    if (errnum == 0)
    {
      proj_context_destroy(ctx);
      throw std::runtime_error("An error occured retrieving the ellipsoid of "
                               "the projects coordinate reference system, "
                               "couldn't identify the error code!");
    }
    std::string err_info(proj_errno_string(errnum));
    proj_context_destroy(ctx);

    throw std::invalid_argument(
        "Could not retrieve the ellipsoid of the projects"
        "projects coordinate reference system: " +
        err_info);
  }

  // get the ellipsoid used in the projects crs
  PJ* ellps_rawptr = proj_get_ellipsoid(ctx, crs_rawptr);

  proj_destroy(crs_rawptr);

  if (ellps_rawptr == nullptr)
  {
    proj_context_destroy(ctx);
    throw std::runtime_error(
        "Could not retrieve the ellipsoid from the PJ object "
        "created from the crs string");
  }

  const char* ellps_projstring_rawptr =
      proj_as_proj_string(ctx, ellps_rawptr, PJ_PROJ_4, nullptr);

  proj_destroy(ellps_rawptr);
  if (ellps_projstring_rawptr == nullptr)
  {
    proj_context_destroy(ctx);
    throw std::runtime_error(
        "Could not convert ellipsoid PJ object to proj string!");
  }

  std::string ellps_string(ellps_projstring_rawptr);
  std::string ellps_specifier("+ellps=");

  size_t ellps_specifier_idx = ellps_string.find(ellps_specifier);
  size_t ellps_idx = ellps_specifier_idx + ellps_specifier.length();

  if (ellps_idx == ellps_string.npos || ellps_idx >= ellps_string.length())
  {
    proj_context_destroy(ctx);
    throw std::runtime_error(
        "Couldn't determine the ellipsoid of the project!");
  }

  std::string ellps = ellps_string.substr(ellps_idx);

  // https://proj.org/en/6.3/operations/conversions/topocentric.html
  // the proj pipeline has two steps, convert geographic to cartesian
  // coordinates then project to topocentric coordinates
  std::string to_topographic_projstr =
      "+proj=pipeline +step +proj=cart +ellps=" + ellps +
      " +step +proj=topocentric" + " +ellps=" + ellps +
      " +lat_0=" + std::to_string(g.latitude) +
      " +lon_0=" + std::to_string(g.longitude) +
      " +h_0=" + std::to_string(g.altitude);

  PJ* to_topographic_rawptr = proj_create(ctx, to_topographic_projstr.c_str());

  if (to_topographic_rawptr == nullptr)
  {
    int errnum = proj_context_errno(ctx);
    if (errnum == 0)
    {
      proj_context_destroy(ctx);
      throw std::runtime_error(
          "An error occured transforming the polygons "
          "coordinates to the projects topographic coordinate system, "
          "couldn't identify the error code!");
    }
    std::string err_info(proj_errno_string(errnum));
    proj_context_destroy(ctx);

    throw std::invalid_argument(
        "Could not get topographic polygon coordinates: " + err_info);
  }

  // apply the transform to all points in transformed_points3d in-place
  // add them to the transformed z vector and insert the transformed vertices
  for (auto&& p : transformed_points3d)
  {
    // here first longitude and then latitude is needed, see
    // https://proj.org/en/stable/development/reference/functions.html#c.proj_create
    // geographic CRS coords have to be passed as long, lat, alt in degrees
    PJ_COORD coord = proj_coord(p.get<1>(), p.get<0>(), p.get<2>(), 0);

    // see
    // https://gis.stackexchange.com/questions/422126/convert-geodetic-to-topocentric-coordinates-in-pyproj
    coord.lpzt.lam = proj_torad(coord.lpzt.lam);
    coord.lpzt.phi = proj_torad(coord.lpzt.phi);

    // PJ_FWD: forward, PJ_IDENT: identity, PJ_INV: inverse
    PJ_COORD t_coord = proj_trans(to_topographic_rawptr, PJ_FWD, coord);

    transformed_polygon.vertices.emplace_back(t_coord.enu.e, t_coord.enu.n);
    transformed_z_coords.push_back(t_coord.enu.u);

    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
        << "Querying with polygon coordinate (in map frame): "
        << " (" << t_coord.enu.e << ", " << t_coord.enu.n << ")";
  }

  // the new z is the smallest z of all transformed points
  transformed_polygon.z = *std::min_element(transformed_z_coords.begin(),
                                            transformed_z_coords.end());

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
      << "Querying with polygons z coordinate (in map frame): "
      << transformed_polygon.z;

  proj_destroy(to_topographic_rawptr);
  proj_context_destroy(ctx);
  return transformed_polygon;
}

seerep_core_msgs::QueryResultProject
CoreProject::getDataset(seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResultProject result;
  result.projectUuid = m_uuid;

  // query.crsString == "project" means the polygon is in the project frame
  // if query.crsString == "map" that means the query is in the map frame
  // else do crs transformations
  if (query.polygon && query.crsString != "map")
  {
    query.polygon.value() =
        transformToMapFrame(query.polygon.value(), query.crsString);
  }

  result.dataOrInstanceUuids = m_coreDatasets->getData(query);

  return result;
}

seerep_core_msgs::QueryResultProject
CoreProject::getInstances(seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResultProject result;
  result.projectUuid = m_uuid;

  // query.crsString == "project" means the polygon is in the project frame
  // if query.crsString == "map" that means the query is done in the map
  // else do crs transformations
  if (query.polygon && query.crsString != "map")
  {
    query.polygon.value() =
        transformToMapFrame(query.polygon.value(), query.crsString);
  }

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

void CoreProject::addLabels(
    const seerep_core_msgs::Datatype& datatype,
    const std::unordered_map<std::string, seerep_core_msgs::LabelDatumaro>&
        labelPerCategory,
    const boost::uuids::uuid& msgUuid)
{
  m_coreDatasets->addLabels(datatype, labelPerCategory, msgUuid);
}

void CoreProject::addTF(const geometry_msgs::TransformStamped& tf)
{
  m_coreTfs->addDataset(tf);
}
std::optional<geometry_msgs::TransformStamped>
CoreProject::getTF(const seerep_core_msgs::QueryTf& transformQuery)
{
  return m_coreTfs->getData(transformQuery.timestamp.seconds,
                            transformQuery.timestamp.nanos,
                            transformQuery.parentFrameId,
                            transformQuery.childFrameId);
}
std::vector<std::string> CoreProject::getFrames()
{
  return m_coreTfs->getFrames();
}

void CoreProject::reinitializeTFs()
{
  m_coreTfs->recreateBufferAndDatasets();
  m_coreDatasets->recreateSpatialRt(seerep_core_msgs::Datatype::Image,
                                    m_ioImage);
  m_coreDatasets->recreateSpatialRt(seerep_core_msgs::Datatype::PointCloud,
                                    m_ioPointCloud);
  m_coreDatasets->recreateSpatialRt(seerep_core_msgs::Datatype::Point,
                                    m_ioPoint);
}

void CoreProject::addCameraIntrinsics(
    const seerep_core_msgs::camera_intrinsics& ci)
{
  m_coreCameraIntrinsics->addData(ci);
}

std::optional<seerep_core_msgs::camera_intrinsics>
CoreProject::getCameraIntrinsics(boost::uuids::uuid camIntrinsicsUuid)
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
  m_hdf5_file = std::make_shared<HighFive::File>(
      path, HighFive::File::ReadWrite | HighFive::File::Create);

  m_ioGeneral = std::make_shared<seerep_hdf5_core::Hdf5CoreGeneral>(
      m_hdf5_file, m_write_mtx);
  m_ioTf =
      std::make_shared<seerep_hdf5_core::Hdf5CoreTf>(m_hdf5_file, m_write_mtx);
  m_ioInstance = std::make_shared<seerep_hdf5_core::Hdf5CoreInstance>(
      m_hdf5_file, m_write_mtx);

  m_ioPointCloud = std::make_shared<seerep_hdf5_core::Hdf5CorePointCloud>(
      m_hdf5_file, m_write_mtx);
  m_ioPoint = std::make_shared<seerep_hdf5_core::Hdf5CorePoint>(m_hdf5_file,
                                                                m_write_mtx);
  m_ioImage = std::make_shared<seerep_hdf5_core::Hdf5CoreImage>(m_hdf5_file,
                                                                m_write_mtx);
}
void CoreProject::recreateDatatypes()
{
  m_coreTfs = std::make_shared<seerep_core::CoreTf>(m_ioTf);
  m_coreCameraIntrinsics = std::make_shared<seerep_core::CoreCameraIntrinsics>(
      m_hdf5_file, m_write_mtx);
  m_coreInstances = std::make_shared<seerep_core::CoreInstances>(m_ioInstance);
  m_coreDatasets = std::make_unique<seerep_core::CoreDataset>(
      m_coreTfs, m_coreInstances, m_frameId);

  m_coreDatasets->addDatatype(seerep_core_msgs::Datatype::Image, m_ioImage);
  m_coreDatasets->addDatatype(seerep_core_msgs::Datatype::PointCloud,
                              m_ioPointCloud);
  m_coreDatasets->addDatatype(seerep_core_msgs::Datatype::Point, m_ioPoint);

  std::vector<std::string> datatypeNames = m_ioGeneral->getGroupDatasets("");
  for (auto datatypeName : datatypeNames)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "found datatype " << datatypeName << " in HDF5 file.";
  }
}

seerep_core_msgs::AabbTime
CoreProject::getTimeBounds(std::vector<seerep_core_msgs::Datatype> datatypes)
{
  return m_coreDatasets->getTimeBounds(datatypes);
}

seerep_core_msgs::AABB
CoreProject::getSpatialBounds(std::vector<seerep_core_msgs::Datatype> datatypes)
{
  return m_coreDatasets->getSpatialBounds(datatypes);
}

std::unordered_set<std::string>
CoreProject::getAllCategories(std::vector<seerep_core_msgs::Datatype> datatypes)
{
  return m_coreDatasets->getAllCategories(datatypes);
}

std::unordered_set<std::string>
CoreProject::getAllLabels(std::vector<seerep_core_msgs::Datatype> datatypes,
                          std::string category)
{
  return m_coreDatasets->getAllLabels(datatypes, category);
}

} /* namespace seerep_core */
