#include "seerep-server/pb-meta-operations.h"

namespace seerep_server
{
PbMetaOperations::PbMetaOperations(std::shared_ptr<seerep_core::Core> seerepCore) : seerepCore(seerepCore)
{
}

grpc::Status PbMetaOperations::CreateProject(grpc::ServerContext* context, const seerep::ProjectCreation* request,
                                             seerep::ProjectInfo* response)
{
  (void)context;  // ignore that variable without causing warnings
  std::cout << "create new project... " << std::endl;
  seerep_core_msgs::ProjectInfo projectInfo;
  projectInfo.frameId = request->mapframeid();
  projectInfo.name = request->name();
  projectInfo.uuid = boost::uuids::random_generator()();

  // assigning gedetic coords attributes individually
  projectInfo.geodetCoords.coordinateSystem = request->geodeticcoordinates().coordinatesystem();
  projectInfo.geodetCoords.ellipsoid = request->geodeticcoordinates().ellipsoid();
  projectInfo.geodetCoords.altitude = request->geodeticcoordinates().altitude();
  projectInfo.geodetCoords.latitude = request->geodeticcoordinates().latitude();
  projectInfo.geodetCoords.longitude = request->geodeticcoordinates().longitude();

  seerepCore->createProject(projectInfo);

  response->set_name(projectInfo.name);
  response->set_uuid(boost::lexical_cast<std::string>(projectInfo.uuid));

  return grpc::Status::OK;
}

grpc::Status PbMetaOperations::GetProjects(grpc::ServerContext* context, const google::protobuf::Empty* request,
                                           seerep::ProjectInfos* response)
{
  (void)context;  // ignore that variable without causing warnings
  (void)request;  // ignore that variable without causing warnings
  std::cout << "query the project infos... " << std::endl;
  auto projectInfos = seerepCore->getProjects();

  for (auto projectInfo : projectInfos)
  {
    auto responseProjectInfo = response->add_projects();
    responseProjectInfo->set_name(projectInfo.name);
    responseProjectInfo->set_uuid(boost::lexical_cast<std::string>(projectInfo.uuid));

    // assigning gedetic coords attributes individually
    // responseProjectInfo->geodeticcoordinates().set_coordinatesystem(projectInfo.geodetCoords.coordinateSystem);
    // responseProjectInfo->geodeticcoordinates().set_ellipsoid(projectInfo.geodetCoords.ellipsoid);
    // responseProjectInfo->geodeticcoordinates()->set_altitude(projectInfo.geodetCoords.altitude);
  }

  return grpc::Status::OK;
}

} /* namespace seerep_server */
