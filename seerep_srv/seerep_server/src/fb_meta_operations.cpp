#include "seerep_server/fb_meta_operations.h"

extern const char* GIT_TAG;

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

  /* TODO: think about adding constructors for the core_msgs structs to shorten this */
  seerep_core_msgs::ProjectInfo projectInfo;
  projectInfo.frameId = requestMsg->map_frame_id()->str();
  projectInfo.name = requestMsg->name()->str();
  projectInfo.version = GIT_TAG;
  projectInfo.uuid = boost::uuids::random_generator()();

  projectInfo.geodetCoords.coordinateSystem = requestMsg->geodetic_position()->coordinateSystem()->str();
  projectInfo.geodetCoords.longitude = requestMsg->geodetic_position()->longitude();
  projectInfo.geodetCoords.latitude = requestMsg->geodetic_position()->latitude();
  projectInfo.geodetCoords.altitude = requestMsg->geodetic_position()->altitude();

  seerepCore->createProject(projectInfo);

  /* TODO: is it necessary to send back all of the data or is a status message sufficient ? */
  flatbuffers::grpc::MessageBuilder builder;
  auto nameOffset = builder.CreateString(projectInfo.name);
  auto uuidOffset = builder.CreateString(boost::lexical_cast<std::string>(projectInfo.uuid));
  auto frameIdOffset = builder.CreateString(projectInfo.frameId);
  auto geodeticPositionOffset =
      seerep::fb::CreateGeodeticCoordinates(builder, builder.CreateString(projectInfo.geodetCoords.coordinateSystem),
                                            projectInfo.geodetCoords.longitude, projectInfo.geodetCoords.latitude,
                                            projectInfo.geodetCoords.altitude);
  auto versionOffset = builder.CreateString(projectInfo.version);
  auto responseOffset = seerep::fb::CreateProjectInfo(builder, nameOffset, uuidOffset, frameIdOffset,
                                                      geodeticPositionOffset, versionOffset);

  builder.Finish(responseOffset);
  *response = builder.ReleaseMessage<seerep::fb::ProjectInfo>();
  assert(response->Verify());

  return grpc::Status::OK;
}

grpc::Status FbMetaOperations::GetProjects(grpc::ServerContext* context,
                                           const flatbuffers::grpc::Message<seerep::fb::Empty>* request,
                                           flatbuffers::grpc::Message<seerep::fb::ProjectInfos>* response)
{
  (void)context;
  (void)request;

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "[FB] querying projects ... ";

  flatbuffers::grpc::MessageBuilder fbb;

  const std::vector<seerep_core_msgs::ProjectInfo>& prjInfos = seerepCore->getProjects();

  auto prjInfosOffset = seerep_core_fb::CoreFbConversion::toFb(fbb, prjInfos);
  fbb.Finish(prjInfosOffset);

  *response = fbb.ReleaseMessage<seerep::fb::ProjectInfos>();
  assert(response->Verify());

  return grpc::Status::OK;
}

grpc::Status FbMetaOperations::LoadProjects(grpc::ServerContext* context,
                                            const flatbuffers::grpc::Message<seerep::fb::Empty>* request,
                                            flatbuffers::grpc::Message<seerep::fb::ProjectInfos>* response)
{
  (void)context;
  (void)request;

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
      << "Loading new projects in the working directory";

  flatbuffers::grpc::MessageBuilder fbb;

  try
  {
    const std::vector<seerep_core_msgs::ProjectInfo>& projectInfos = seerepCore->loadProjectsInFolder();
    auto prjInfosOffset = seerep_core_fb::CoreFbConversion::toFb(fbb, projectInfos);

    fbb.Finish(prjInfosOffset);
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

  *response = fbb.ReleaseMessage<seerep::fb::ProjectInfos>();
  assert(response->Verify());

  return grpc::Status::OK;
}

grpc::Status FbMetaOperations::DeleteProject(grpc::ServerContext* context,
                                             const flatbuffers::grpc::Message<seerep::fb::ProjectInfo>* request,
                                             flatbuffers::grpc::Message<seerep::fb::Empty>* response)
{
  (void)context;  // ignore that variable without causing warnings
  (void)request;  // ignore that variable without causing warnings

  try
  {
    const std::string projectUuid = request->GetRoot()->uuid()->str();
    seerepCore->deleteProject(boost::uuids::string_generator()(projectUuid));
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "Deleted project with uuid: " << projectUuid;
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

grpc::Status
FbMetaOperations::GetOverallTimeInterval(grpc::ServerContext* context,
                                         const flatbuffers::grpc::Message<seerep::fb::UuidDatatypePair>* request,
                                         flatbuffers::grpc::Message<seerep::fb::TimeInterval>* response)
{
  (void)context;  // ignore that variable without causing warnings
  auto requestRoot = request->GetRoot();

  // It was not possible to send a vector of enum of datatypes
  // through to this function. Therefore, on the flatbuffer service
  // level, the UuidDatatypePair implementation has one datatype,
  // while below this layer, in the seerep core, the implementation
  // is a vector. We intend to make them consistent in the future
  // by implementing a vector of enums on the flatbuffer (and protobuf)
  // levels.
  std::vector<seerep_core_msgs::Datatype> dtype_vec =
      seerep_core_fb::CoreFbConversion::fromFbDatatypeVector(requestRoot->datatype());

  try
  {
    flatbuffers::grpc::MessageBuilder fbb;
    boost::uuids::uuid projectUuid = boost::uuids::string_generator()(requestRoot->projectuuid()->str());

    seerep_core_msgs::AabbTime timeinterval = seerepCore->getOverallTimeInterval(projectUuid, dtype_vec);
    flatbuffers::Offset<seerep::fb::TimeInterval> bb = seerep_core_fb::CoreFbConversion::toFb(fbb, timeinterval);

    fbb.Finish(bb);
    *response = fbb.ReleaseMessage<seerep::fb::TimeInterval>();
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

  return grpc::Status::OK;
}

grpc::Status
FbMetaOperations::GetOverallBoundingBox(grpc::ServerContext* context,
                                        const flatbuffers::grpc::Message<seerep::fb::UuidDatatypePair>* request,
                                        flatbuffers::grpc::Message<seerep::fb::Boundingbox>* response)
{
  (void)context;  // ignore that variable without causing warnings
  auto requestRoot = request->GetRoot();

  // It was not possible to send a vector of enum of datatypes
  // through to this function. Therefore, on the flatbuffer service
  // level, the UuidDatatypePair implementation has one datatype,
  // while below this layer, in the seerep core, the implementation
  // is a vector. We intend to make them consistent in the future
  // by implementing a vector of enums on the flatbuffer (and protobuf)
  // levels.
  std::vector<seerep_core_msgs::Datatype> dtype_vec =
      seerep_core_fb::CoreFbConversion::fromFbDatatypeVector(requestRoot->datatype());

  try
  {
    flatbuffers::grpc::MessageBuilder fbb;
    boost::uuids::uuid projectUuid = boost::uuids::string_generator()(requestRoot->projectuuid()->str());

    seerep_core_msgs::AABB overallBB = seerepCore->getOverallBound(projectUuid, dtype_vec);
    flatbuffers::Offset<seerep::fb::Boundingbox> bb = seerep_core_fb::CoreFbConversion::toFb(fbb, overallBB);

    fbb.Finish(bb);
    *response = fbb.ReleaseMessage<seerep::fb::Boundingbox>();
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

  return grpc::Status::OK;
}

grpc::Status FbMetaOperations::GetAllCategories(grpc::ServerContext* context,
                                                const flatbuffers::grpc::Message<seerep::fb::UuidDatatypePair>* request,
                                                flatbuffers::grpc::Message<seerep::fb::StringVector>* response)
{
  (void)context;  // ignore that variable without causing warnings
  auto requestRoot = request->GetRoot();

  // It was not possible to send a vector of enum of datatypes
  // through to this function. Therefore, on the flatbuffer service
  // level, the UuidDatatypePair implementation has one datatype,
  // while below this layer, in the seerep core, the implementation
  // is a vector. We intend to make them consistent in the future
  // by implementing a vector of enums on the flatbuffer (and protobuf)
  // levels.
  std::vector<seerep_core_msgs::Datatype> datatype_vec =
      seerep_core_fb::CoreFbConversion::fromFbDatatypeVector(requestRoot->datatype());

  try
  {
    flatbuffers::grpc::MessageBuilder fbb;
    boost::uuids::uuid projectUuid = boost::uuids::string_generator()(requestRoot->projectuuid()->str());

    std::unordered_set<std::string> categories = seerepCore->getAllCategories(projectUuid, datatype_vec);
    std::vector<flatbuffers::Offset<flatbuffers::String>> fb_categories;
    fb_categories.reserve(categories.size());

    for (const std::string& category : categories)
    {
      fb_categories.push_back(fbb.CreateString(category));
    }

    fbb.Finish(seerep::fb::CreateStringVector(fbb, fbb.CreateVector(fb_categories)));

    *response = fbb.ReleaseMessage<seerep::fb::StringVector>();
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

  return grpc::Status::OK;
}

grpc::Status
FbMetaOperations::GetAllLabels(grpc::ServerContext* context,
                               const flatbuffers::grpc::Message<seerep::fb::UuidDatatypeWithCategory>* request,
                               flatbuffers::grpc::Message<seerep::fb::StringVector>* response)
{
  (void)context;  // ignore that variable without causing warnings
  auto requestRoot = request->GetRoot();

  // It was not possible to send a vector of enum of datatypes
  // through to this function. Therefore, on the flatbuffer service
  // level, the UuidDatatypePair implementation has one datatype,
  // while below this layer, in the seerep core, the implementation
  // is a vector. We intend to make them consistent in the future
  // by implementing a vector of enums on the flatbuffer (and protobuf)
  // levels.
  std::vector<seerep_core_msgs::Datatype> dtype_vec =
      seerep_core_fb::CoreFbConversion::fromFbDatatypeVector(requestRoot->UuidAndDatatype()->datatype());

  try
  {
    flatbuffers::grpc::MessageBuilder fbb;
    boost::uuids::uuid projectUuid =
        boost::uuids::string_generator()(requestRoot->UuidAndDatatype()->projectuuid()->str());

    std::unordered_set<std::string> labels =
        seerepCore->getAllLabels(projectUuid, dtype_vec, requestRoot->category()->str());
    std::vector<flatbuffers::Offset<flatbuffers::String>> fb_labels;
    fb_labels.reserve(labels.size());

    for (const std::string& label : labels)
    {
      fb_labels.push_back(fbb.CreateString(label));
    }

    fbb.Finish(seerep::fb::CreateStringVector(fbb, fbb.CreateVector(fb_labels)));

    *response = fbb.ReleaseMessage<seerep::fb::StringVector>();
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
  return grpc::Status::OK;
}

} /* namespace seerep_server */
