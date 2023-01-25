#include "seerep_server/pb_meta_operations.h"

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
                                                      const seerep::UuidDatatypePair* request,
                                                      seerep::TimeInterval* response)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "fetching overall time interval";

  std::vector<seerep_core_msgs::Datatype> dt_vector;

  if (request->datatype() == seerep::datatype::image)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Image);
  }
  else if (request->datatype() == seerep::datatype::pointcloud)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::PointCloud);
  }
  else if (request->datatype() == seerep::datatype::point)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Point);
  }
  else
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Unknown);
  }

  try
  {
    std::string uuid = request->projectuuid();
    boost::uuids::string_generator gen;
    auto uuidFromString = gen(uuid);

    seerep_core_msgs::AabbTime timeinterval = seerepCore->getOverallTimeInterval(uuidFromString, dt_vector);

    // isolate second and nano second bits from min time
    uint64_t mintime = timeinterval.min_corner().get<0>();
    uint32_t min_nanos = (uint32_t)mintime;
    uint32_t min_seconds = (uint32_t)(mintime >> 32);

    // isolate second and nano second bits from max time
    uint64_t maxtime = timeinterval.max_corner().get<0>();
    uint32_t max_nanos = (uint32_t)maxtime;
    uint32_t max_seconds = (uint32_t)(maxtime >> 32);

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
                                                     const seerep::UuidDatatypePair* request,
                                                     seerep::Boundingbox* response)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "fetching overall bounding box";

  std::vector<seerep_core_msgs::Datatype> dt_vector;

  if (request->datatype() == seerep::datatype::image)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Image);
  }
  else if (request->datatype() == seerep::datatype::pointcloud)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::PointCloud);
  }
  else if (request->datatype() == seerep::datatype::point)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Point);
  }
  else
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Unknown);
  }

  try
  {
    std::string uuid = request->projectuuid();
    boost::uuids::string_generator gen;
    auto uuidFromString = gen(uuid);

    seerep_core_msgs::AABB overallBB = seerepCore->getOverallBound(uuidFromString, dt_vector);

    flatbuffers::grpc::MessageBuilder builder;

    response->mutable_point_min()->set_x(overallBB.min_corner().get<0>());
    response->mutable_point_min()->set_y(overallBB.min_corner().get<1>());
    response->mutable_point_min()->set_z(overallBB.min_corner().get<2>());

    response->mutable_point_max()->set_x(overallBB.max_corner().get<0>());
    response->mutable_point_max()->set_y(overallBB.max_corner().get<1>());
    response->mutable_point_max()->set_z(overallBB.max_corner().get<2>());
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

} /* namespace seerep_server */
