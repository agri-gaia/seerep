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
  try
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "create new project... ";
    seerep_core_msgs::ProjectInfo projectInfo;
    projectInfo.frameId = request->mapframeid();
    projectInfo.name = request->name();
    projectInfo.uuid = boost::uuids::random_generator()();
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
                                           seerep::ProjectInfos* response)
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

grpc::Status GetOverallTimeInterval(grpc::ServerContext* context, const seerep::UuidDatatypePair* request,
                                    seerep::TimeInterval* response)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "fetching overall time interval";

  std::string uuid = request->projectuuid();
  boost::uuids::string_generator gen;
  auto uuidFromString = gen(uuid);

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

  seerep_core_msgs::AabbTime timeinterval = seerepCore->getOverallTimeInterval(uuidFromString, dt_vector);

  // isolate second and nano second bits from min time
  uint64_t mintime = timeinterval.min_corner().get<0>();
  uint32_t min_nanos = (uint32_t)mintime;
  uint32_t min_seconds = (uint32_t)(mintime >> 32);

  // isolate second and nano second bits from max time
  uint64_t maxtime = timeinterval.max_corner().get<0>();
  uint32_t max_nanos = (uint32_t)maxtime;
  uint32_t max_seconds = (uint32_t)(maxtime >> 32);

  response->set_time_min(mintime);
  response->set_time_max(maxtime);

  return grpc::Status::OK;
}
grpc::Status GetOverallBoundingBox(grpc::ServerContext* context, const seerep::UuidDatatypePair* request,
                                   seerep::Boundingbox* response)
{
  (void)context;  // ignore that variable without causing warnings
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "fetching overall bounding box";

  std::string uuid = request->projectuuid();
  boost::uuids::string_generator gen;
  auto uuidFromString = gen(uuid);

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

  seerep_core_msgs::AABB overallBB = seerepCore->getOverallBound(uuidFromString, dt_vector);

  flatbuffers::grpc::MessageBuilder builder;

  response->set_min_corner(overallBB.min_corner());
  response->set_max_corner(overallBB.max_corner());

  return grpc::Status::OK;
}

} /* namespace seerep_server */
