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

} /* namespace seerep_server */
