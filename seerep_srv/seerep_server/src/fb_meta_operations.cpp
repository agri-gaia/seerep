#include "seerep_server/fb_meta_operations.h"

namespace seerep_server
{
FbMetaOperations::FbMetaOperations(std::shared_ptr<seerep_core::Core> seerepCore) : seerepCore(seerepCore)
{
}

grpc::Status FbMetaOperations::CreateProject(grpc::ServerContext* context,
                                             const flatbuffers::grpc::Message<seerep::fb::ProjectCreation>* request,
                                             flatbuffers::grpc::Message<seerep::fb::ProjectInfo>* response)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "create new project... ";
  const seerep::fb::ProjectCreation* requestMsg = request->GetRoot();
  seerep_core_msgs::ProjectInfo projectInfo;
  projectInfo.frameId = requestMsg->map_frame_id()->str();
  projectInfo.name = requestMsg->name()->str();
  projectInfo.uuid = boost::uuids::random_generator()();

  // extracting geodetic coordinates attribute information from flatbuffer and saving in seerep core msg struct
  projectInfo.geodetCoords.coordinateSystem = requestMsg->geodetic_position()->coordinateSystem()->str();
  projectInfo.geodetCoords.ellipsoid = requestMsg->geodetic_position()->ellipsoid()->str();
  projectInfo.geodetCoords.longitude = requestMsg->geodetic_position()->longitude();
  projectInfo.geodetCoords.latitude = requestMsg->geodetic_position()->latitude();
  projectInfo.geodetCoords.altitude = requestMsg->geodetic_position()->altitude();

  seerepCore->createProject(projectInfo);

  flatbuffers::grpc::MessageBuilder builder;
  auto nameOffset = builder.CreateString(projectInfo.name);
  auto uuidOffset = builder.CreateString(boost::lexical_cast<std::string>(projectInfo.uuid));
  auto frameIdOffset = builder.CreateString(projectInfo.frameId);
  auto responseOffset = seerep::fb::CreateProjectInfo(builder, nameOffset, uuidOffset, frameIdOffset);

  builder.Finish(responseOffset);
  *response = builder.ReleaseMessage<seerep::fb::ProjectInfo>();
  assert(response->Verify());

  return grpc::Status::OK;
}

grpc::Status FbMetaOperations::GetProjects(grpc::ServerContext* context,
                                           const flatbuffers::grpc::Message<seerep::fb::Empty>* request,
                                           flatbuffers::grpc::Message<seerep::fb::ProjectInfos>* response)
{
  (void)context;  // ignore that variable without causing warnings
  (void)request;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "query the project infos... ";
  auto projectInfos = seerepCore->getProjects();

  flatbuffers::grpc::MessageBuilder builder;
  std::vector<flatbuffers::Offset<seerep::fb::ProjectInfo>> projectInfosVector;
  for (auto projectInfo : projectInfos)
  {
    auto nameOffset = builder.CreateString(projectInfo.name);
    auto uuidOffset = builder.CreateString(boost::lexical_cast<std::string>(projectInfo.uuid));
    auto frameIdOffset = builder.CreateString(projectInfo.frameId);

    auto coordinateSystemOffset = builder.CreateString(projectInfo.geodetCoords.coordinateSystem);
    auto ellipsoidOffset = builder.CreateString(projectInfo.geodetCoords.ellipsoid);

    seerep::fb::GeodeticCoordinatesBuilder gcbuilder(builder);
    gcbuilder.add_coordinateSystem(coordinateSystemOffset);
    gcbuilder.add_ellipsoid(ellipsoidOffset);
    gcbuilder.add_altitude(projectInfo.geodetCoords.altitude);
    gcbuilder.add_latitude(projectInfo.geodetCoords.latitude);
    gcbuilder.add_longitude(projectInfo.geodetCoords.longitude);
    auto geodeticCoordinatesOffset = gcbuilder.Finish();

    projectInfosVector.push_back(
        seerep::fb::CreateProjectInfo(builder, nameOffset, uuidOffset, frameIdOffset, geodeticCoordinatesOffset));
  }
  auto vectorOffset = builder.CreateVector(projectInfosVector);
  auto projectInfosOffset = seerep::fb::CreateProjectInfos(builder, vectorOffset);
  builder.Finish(projectInfosOffset);
  *response = builder.ReleaseMessage<seerep::fb::ProjectInfos>();
  assert(response->Verify());

  return grpc::Status::OK;
}

grpc::Status FbMetaOperations::LoadProjects(grpc::ServerContext* context,
                                            const flatbuffers::grpc::Message<seerep::fb::Empty>* request,
                                            flatbuffers::grpc::Message<seerep::fb::Empty>* response)
{
  (void)context;  // ignore that variable without causing warnings
  (void)request;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
      << "loading all projects in the working directory... ";

  try
  {
    seerepCore->loadProjectsInFolder();
  }
  catch (std::runtime_error const& e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
    // also catching core doesn't have project with uuid error
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
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

  flatbuffers::grpc::MessageBuilder builder;
  seerep::fb::CreateEmpty(builder);
  *response = builder.ReleaseMessage<seerep::fb::Empty>();

  return grpc::Status::OK;
}

grpc::Status FbMetaOperations::DeleteProject(grpc::ServerContext* context,
                                             const flatbuffers::grpc::Message<seerep::fb::ProjectInfo>* request,
                                             flatbuffers::grpc::Message<seerep::fb::Empty>* response)
{
  (void)context;  // ignore that variable without causing warnings
  (void)request;  // ignore that variable without causing warnings
  std::string uuid = request->GetRoot()->uuid()->str();
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "deleting project... with uuid: " << uuid;

  try
  {
    boost::uuids::string_generator gen;
    auto uuidFromString = gen(uuid);
    seerepCore->deleteProject(uuidFromString);
  }
  catch (std::runtime_error const& e)
  {
    // mainly catching "invalid uuid string" when transforming uuid_project from string to uuid
    // also catching core doesn't have project with uuid error
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, e.what());
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
  flatbuffers::grpc::MessageBuilder builder;
  seerep::fb::CreateEmpty(builder);
  *response = builder.ReleaseMessage<seerep::fb::Empty>();

  return grpc::Status::OK;
}

// grpc::Status
// FbMetaOperations::GetOverallTimeInterval(grpc::ServerContext* context,
//                                          const flatbuffers::grpc::Message<seerep::fb::UuidDatatypePair>* request,
//                                          flatbuffers::grpc::Message<seerep::fb::TimeInterval>* response)
// {
//   std::string uuid = request->GetRoot()->projectuuid()->str();
//   boost::uuids::string_generator gen;
//   auto uuidFromString = gen(uuid);

//   std::vector<std::string> dt_vector;
//   for (auto dt : *request->GetRoot()->datatypes())
//   {
//     dt_vector.push_back(dt->str());
//   }

//   seerep_core_msgs::AabbTime timeinterval = seerepCore->getOverallTimeInterval(uuidFromString);

//   flatbuffers::grpc::MessageBuilder builder;

//   seerep::fb::TimestampBuilder minTimeStampBuilder(builder);
//   minTimeStampBuilder.add_seconds(timeinterval.min_corner().get<0>());
//   flatbuffers::Offset<seerep::fb::Timestamp> min = minTimeStampBuilder.Finish();

//   seerep::fb::TimestampBuilder maxTimeStampBuilder(builder);
//   maxTimeStampBuilder.add_seconds(timeinterval.max_corner().get<0>());
//   flatbuffers::Offset<seerep::fb::Timestamp> max = maxTimeStampBuilder.Finish();

//   seerep::fb::TimeIntervalBuilder timeIntervalBuilder(builder);
//   timeIntervalBuilder.add_time_min(min);
//   timeIntervalBuilder.add_time_max(max);
//   flatbuffers::Offset<seerep::fb::TimeInterval> bb = timeIntervalBuilder.Finish();

//   builder.Finish(bb);
//   *response = builder.ReleaseMessage<seerep::fb::TimeInterval>();

//   return grpc::Status::OK;
// }

grpc::Status
FbMetaOperations::GetOverallBoundingBox(grpc::ServerContext* context,
                                        const flatbuffers::grpc::Message<seerep::fb::UuidDatatypePair>* request,
                                        flatbuffers::grpc::Message<seerep::fb::Boundingbox>* response)
{
  (void)context;  // ignore that variable without causing warnings
  auto requestRoot = request->GetRoot();

  std::string uuid = requestRoot->projectuuid()->str();
  boost::uuids::string_generator gen;
  auto uuidFromString = gen(uuid);

  std::vector<seerep_core_msgs::Datatype> dt_vector;

  seerep::fb::Datatype casted_datatype = static_cast<seerep::fb::Datatype>(requestRoot->datatypes());
  if (casted_datatype == seerep::fb::Datatype_Image)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Image);
  }
  else if (casted_datatype == seerep::fb::Datatype_PointCloud)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::PointCloud);
  }
  else if (casted_datatype == seerep::fb::Datatype_Point)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Point);
  }
  else
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Unknown);
  }

  seerep_core_msgs::AABB overallBB = seerepCore->getOverallBound(uuidFromString, dt_vector);

  flatbuffers::grpc::MessageBuilder builder;

  seerep::fb::PointBuilder minPointBuilder(builder);
  minPointBuilder.add_x(overallBB.min_corner().get<0>());
  minPointBuilder.add_y(overallBB.min_corner().get<1>());
  minPointBuilder.add_z(overallBB.min_corner().get<2>());
  flatbuffers::Offset<seerep::fb::Point> minPoint = minPointBuilder.Finish();

  seerep::fb::PointBuilder maxPointBuilder(builder);
  maxPointBuilder.add_x(overallBB.max_corner().get<0>());
  maxPointBuilder.add_y(overallBB.max_corner().get<1>());
  maxPointBuilder.add_z(overallBB.max_corner().get<2>());
  flatbuffers::Offset<seerep::fb::Point> maxPoint = maxPointBuilder.Finish();

  seerep::fb::BoundingboxBuilder boundingBoxBuilder(builder);
  boundingBoxBuilder.add_point_min(minPoint);
  boundingBoxBuilder.add_point_max(maxPoint);
  flatbuffers::Offset<seerep::fb::Boundingbox> bb = boundingBoxBuilder.Finish();

  builder.Finish(bb);
  *response = builder.ReleaseMessage<seerep::fb::Boundingbox>();

  return grpc::Status::OK;
}

} /* namespace seerep_server */
