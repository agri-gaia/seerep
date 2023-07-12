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
  seerep_core_msgs::ProjectInfo projectInfo;
  projectInfo.frameId = requestMsg->map_frame_id()->str();
  projectInfo.name = requestMsg->name()->str();
  projectInfo.version = GIT_TAG;
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
      << "[FB] loading new projects in the working directory ... ";

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
  std::vector<seerep_core_msgs::Datatype> dt_vector;

  dt_vector = seerep_core_fb::CoreFbConversion::fromFbDatatypeVector(requestRoot->datatype());

  try
  {
    std::string uuid = request->GetRoot()->projectuuid()->str();
    boost::uuids::string_generator gen;
    auto uuidFromString = gen(uuid);

    seerep_core_msgs::AabbTime timeinterval = seerepCore->getOverallTimeInterval(uuidFromString, dt_vector);

    flatbuffers::grpc::MessageBuilder builder;
    flatbuffers::Offset<seerep::fb::TimeInterval> bb = seerep_core_fb::CoreFbConversion::toFb(builder, timeinterval);

    builder.Finish(bb);
    *response = builder.ReleaseMessage<seerep::fb::TimeInterval>();
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
  std::vector<seerep_core_msgs::Datatype> dt_vector;

  dt_vector = seerep_core_fb::CoreFbConversion::fromFbDatatypeVector(requestRoot->datatype());

  try
  {
    std::string uuid = requestRoot->projectuuid()->str();
    boost::uuids::string_generator gen;
    auto uuidFromString = gen(uuid);

    seerep_core_msgs::AABB overallBB = seerepCore->getOverallBound(uuidFromString, dt_vector);

    flatbuffers::grpc::MessageBuilder builder;

    flatbuffers::Offset<seerep::fb::Boundingbox> bb = seerep_core_fb::CoreFbConversion::toFb(builder, overallBB);

    builder.Finish(bb);
    *response = builder.ReleaseMessage<seerep::fb::Boundingbox>();
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
                                                flatbuffers::grpc::Message<seerep::fb::Categories>* response)
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
  std::vector<seerep_core_msgs::Datatype> dt_vector;

  dt_vector = seerep_core_fb::CoreFbConversion::fromFbDatatypeVector(requestRoot->datatype());

  try
  {
    std::string uuid = requestRoot->projectuuid()->str();
    boost::uuids::string_generator gen;
    auto uuidFromString = gen(uuid);

    std::unordered_set<std::string> categories = seerepCore->getAllCategories(uuidFromString, dt_vector);

    flatbuffers::grpc::MessageBuilder builder;

    std::vector<flatbuffers::Offset<flatbuffers::String>> fb_categories;

    for (std::string cat : categories)
    {
      flatbuffers::Offset<flatbuffers::String> fb_category = builder.CreateString(cat);
      fb_categories.push_back(fb_category);
    }

    auto fb_categories_vector = builder.CreateVector(fb_categories);

    seerep::fb::CategoriesBuilder cb(builder);
    cb.add_categories(fb_categories_vector);
    builder.Finish(cb.Finish());

    *response = builder.ReleaseMessage<seerep::fb::Categories>();
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
                               flatbuffers::grpc::Message<seerep::fb::Labels>* response)
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
  std::vector<seerep_core_msgs::Datatype> dt_vector;

  dt_vector = seerep_core_fb::CoreFbConversion::fromFbDatatypeVector(requestRoot->UuidAndDatatype()->datatype());

  try
  {
    std::string category = requestRoot->category()->str();

    std::string uuid = requestRoot->UuidAndDatatype()->projectuuid()->str();
    boost::uuids::string_generator gen;
    auto uuidFromString = gen(uuid);

    std::unordered_set<std::string> allLabels = seerepCore->getAllLabels(uuidFromString, dt_vector, category);

    flatbuffers::grpc::MessageBuilder builder;

    std::vector<flatbuffers::Offset<flatbuffers::String>> fb_labels;

    for (std::string lbl : allLabels)
    {
      flatbuffers::Offset<flatbuffers::String> fb_label = builder.CreateString(lbl);
      fb_labels.push_back(fb_label);
    }

    auto fb_labels_vector = builder.CreateVector(fb_labels);

    seerep::fb::LabelsBuilder lb(builder);
    lb.add_labels(fb_labels_vector);
    builder.Finish(lb.Finish());

    *response = builder.ReleaseMessage<seerep::fb::Labels>();
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
