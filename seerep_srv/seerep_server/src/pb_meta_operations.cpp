#include "seerep_server/pb_meta_operations.h"

extern const char* GIT_TAG;

namespace seerep_server
{
PbMetaOperations::PbMetaOperations(std::shared_ptr<seerep_core::Core> seerepCore) : seerepCore(seerepCore)
{
}

grpc::Status PbMetaOperations::CreateProject(grpc::ServerContext* context, const seerep::pb::ProjectCreation* request,
                                             seerep::pb::ProjectInfo* response)
{
  (void)context;  // ignore that variable without causing warnings
  try
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "create new project... ";

    seerep_core_msgs::ProjectInfo projectInfo;
    projectInfo.frameId = request->mapframeid();
    projectInfo.name = request->name();
    projectInfo.version = GIT_TAG;
    projectInfo.uuid = boost::uuids::random_generator()();

    // assigning geodetic coords attributes individually
    projectInfo.geodetCoords.coordinateSystem = request->geodeticcoordinates().coordinatesystem();
    projectInfo.geodetCoords.ellipsoid = request->geodeticcoordinates().ellipsoid();
    projectInfo.geodetCoords.altitude = request->geodeticcoordinates().altitude();
    projectInfo.geodetCoords.latitude = request->geodeticcoordinates().latitude();
    projectInfo.geodetCoords.longitude = request->geodeticcoordinates().longitude();

    seerepCore->createProject(projectInfo);

    response->set_name(projectInfo.name);
    response->set_uuid(boost::lexical_cast<std::string>(projectInfo.uuid));
    response->set_frameid(projectInfo.frameId);
  }
  catch (const std::exception& e)
  {
    // specific handling for all exceptions extending std::exception, except
    // std::runtime_error which is handled explicitly
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (...)
  {
    // catch any other errors (that we have no information about)
    std::string msg = "Unknown failure occurred. Possible memory corruption";
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
  }
  return grpc::Status::OK;
}

grpc::Status PbMetaOperations::GetProjects(grpc::ServerContext* context, const google::protobuf::Empty* request,
                                           seerep::pb::ProjectInfos* response)
{
  (void)context;  // ignore that variable without causing warnings
  (void)request;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "query the project infos... ";

  try
  {
    auto projectInfos = seerepCore->getProjects();

    for (auto projectInfo : projectInfos)
    {
      auto responseProjectInfo = response->add_projects();
      responseProjectInfo->set_name(projectInfo.name);
      responseProjectInfo->set_uuid(boost::lexical_cast<std::string>(projectInfo.uuid));
      responseProjectInfo->set_frameid(projectInfo.frameId);
      responseProjectInfo->set_version(projectInfo.version);

      // assigning geodetic coords attributes individually
      responseProjectInfo->mutable_geodeticcoordinates()->set_coordinatesystem(
          projectInfo.geodetCoords.coordinateSystem);
      responseProjectInfo->mutable_geodeticcoordinates()->set_ellipsoid(projectInfo.geodetCoords.ellipsoid);
      responseProjectInfo->mutable_geodeticcoordinates()->set_altitude(projectInfo.geodetCoords.altitude);
      responseProjectInfo->mutable_geodeticcoordinates()->set_latitude(projectInfo.geodetCoords.latitude);
      responseProjectInfo->mutable_geodeticcoordinates()->set_longitude(projectInfo.geodetCoords.longitude);
    }
  }
  catch (const std::exception& e)
  {
    // specific handling for all exceptions extending std::exception, except
    // std::runtime_error which is handled explicitly
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (...)
  {
    // catch any other errors (that we have no information about)
    std::string msg = "Unknown failure occurred. Possible memory corruption";
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
  }

  return grpc::Status::OK;
}

grpc::Status PbMetaOperations::GetOverallTimeInterval(grpc::ServerContext* context,
                                                      const seerep::pb::UuidDatatypePair* request,
                                                      seerep::pb::TimeInterval* response)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "fetching overall time interval";

  std::vector<seerep_core_msgs::Datatype> dt_vector;
  dt_vector = convertPbDatatypeVector(request->datatype());

  try
  {
    std::string uuid = request->projectuuid();
    boost::uuids::string_generator gen;
    auto uuidFromString = gen(uuid);

    seerep_core_msgs::AabbTime timeinterval = seerepCore->getOverallTimeInterval(uuidFromString, dt_vector);

    // isolate second and nano second bits from min time
    uint64_t mintime = timeinterval.min_corner().get<0>();
    uint32_t min_nanos = (uint32_t)mintime;
    uint32_t min_seconds = (int32_t)(mintime >> 32);

    // isolate second and nano second bits from max time
    uint64_t maxtime = timeinterval.max_corner().get<0>();
    uint32_t max_nanos = (uint32_t)maxtime;
    uint32_t max_seconds = (int32_t)(maxtime >> 32);

    response->mutable_time_min()->set_nanos(min_nanos);
    response->mutable_time_min()->set_seconds(min_seconds);

    response->mutable_time_max()->set_nanos(max_nanos);
    response->mutable_time_max()->set_seconds(max_seconds);
  }
  catch (const std::exception& e)
  {
    // specific handling for all exceptions extending std::exception, except
    // std::runtime_error which is handled explicitly
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (...)
  {
    // catch any other errors (that we have no information about)
    std::string msg = "Unknown failure occurred. Possible memory corruption";
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
  }

  return grpc::Status::OK;
}
grpc::Status PbMetaOperations::GetOverallBoundingBox(grpc::ServerContext* context,
                                                     const seerep::pb::UuidDatatypePair* request,
                                                     seerep::pb::Boundingbox* response)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "fetching overall bounding box";

  std::vector<seerep_core_msgs::Datatype> dt_vector;
  dt_vector = convertPbDatatypeVector(request->datatype());

  try
  {
    std::string uuid = request->projectuuid();
    boost::uuids::string_generator gen;
    auto uuidFromString = gen(uuid);

    seerep_core_msgs::AABB overallBB = seerepCore->getOverallBound(uuidFromString, dt_vector);

    // center
    int center_x = (overallBB.min_corner().get<0>() + overallBB.max_corner().get<0>()) / 2;
    int center_y = (overallBB.min_corner().get<1>() + overallBB.max_corner().get<1>()) / 2;
    int center_z = (overallBB.min_corner().get<2>() + overallBB.max_corner().get<2>()) / 2;

    // spatial extent
    int se_x = (overallBB.max_corner().get<0>() - overallBB.min_corner().get<0>());
    int se_y = (overallBB.max_corner().get<1>() - overallBB.min_corner().get<1>());
    int se_z = (overallBB.max_corner().get<2>() - overallBB.min_corner().get<2>());

    flatbuffers::grpc::MessageBuilder builder;

    response->mutable_center_point()->set_x(center_x);
    response->mutable_center_point()->set_y(center_y);
    response->mutable_center_point()->set_z(center_z);

    response->mutable_spatial_extent()->set_x(se_x);
    response->mutable_spatial_extent()->set_y(se_y);
    response->mutable_spatial_extent()->set_z(se_z);
  }
  catch (const std::exception& e)
  {
    // specific handling for all exceptions extending std::exception, except
    // std::runtime_error which is handled explicitly
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (...)
  {
    // catch any other errors (that we have no information about)
    std::string msg = "Unknown failure occurred. Possible memory corruption";
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
  }

  return grpc::Status::OK;
}

grpc::Status PbMetaOperations::GetAllCategories(grpc::ServerContext* context,
                                                const seerep::pb::UuidDatatypePair* request,
                                                seerep::pb::Categories* response)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "fetching all categories";

  std::vector<seerep_core_msgs::Datatype> dt_vector;
  dt_vector = convertPbDatatypeVector(request->datatype());

  try
  {
    std::string uuid = request->projectuuid();
    boost::uuids::string_generator gen;
    auto uuidFromString = gen(uuid);

    std::vector<std::string> categories = seerepCore->getAllCategories(uuidFromString, dt_vector);

    for (std::string category : categories)
    {
      response->add_categories(category);
    }
  }
  catch (const std::exception& e)
  {
    // specific handling for all exceptions extending std::exception, except
    // std::runtime_error which is handled explicitly
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (...)
  {
    // catch any other errors (that we have no information about)
    std::string msg = "Unknown failure occurred. Possible memory corruption";
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
  }

  return grpc::Status::OK;
}

grpc::Status PbMetaOperations::GetAllLabels(grpc::ServerContext* context,
                                            const seerep::pb::UuidDatatypeWithCategory* request,
                                            seerep::pb::Labels* response)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "fetching overall bounding box";

  std::vector<seerep_core_msgs::Datatype> dt_vector;
  dt_vector = convertPbDatatypeVector(request->uuid_with_datatype().datatype());

  try
  {
    std::string category = request->category();

    std::string uuid = request->uuid_with_datatype().projectuuid();
    boost::uuids::string_generator gen;
    auto uuidFromString = gen(uuid);

    std::vector<std::string> allLabels = seerepCore->getAllLabels(uuidFromString, dt_vector, category);

    for (std::string label : allLabels)
    {
      response->add_labels(label);
    }
  }
  catch (const std::exception& e)
  {
    // specific handling for all exceptions extending std::exception, except
    // std::runtime_error which is handled explicitly
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
  }
  catch (...)
  {
    // catch any other errors (that we have no information about)
    std::string msg = "Unknown failure occurred. Possible memory corruption";
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << msg;
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, msg);
  }

  return grpc::Status::OK;
}

std::vector<seerep_core_msgs::Datatype> PbMetaOperations::convertPbDatatypeVector(const seerep::datatype datatype)
{
  std::vector<seerep_core_msgs::Datatype> dt_vector;

  if (datatype == seerep::datatype::image)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Image);
  }
  else if (datatype == seerep::datatype::pointcloud)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::PointCloud);
  }
  else if (datatype == seerep::datatype::point)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Point);
  }
  else if (datatype == seerep::datatype::all)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Image);
    dt_vector.push_back(seerep_core_msgs::Datatype::PointCloud);
    dt_vector.push_back(seerep_core_msgs::Datatype::Point);
  }
  else
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Unknown);
  }

  return dt_vector;
}

} /* namespace seerep_server */
