#include "seerep-server/server.h"

namespace seerep_server
{
server::server(std::shared_ptr<seerep_core::Core> seerepCore) : m_seerepCore(seerepCore)
{
}

void server::addServicesPb(grpc::ServerBuilder& server_builder)
{
  // create services
  createServicesPb();
  // add services
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "add the protobuf gRPC services...";
  server_builder.RegisterService(&*m_metaOperationsPb);
  server_builder.RegisterService(&*m_tfServicePb);
  server_builder.RegisterService(&*m_imageServicePb);
  server_builder.RegisterService(&*m_pointCloudServicePb);
  // server_builder.RegisterService(receiveSensorMsgs);
}

void server::addServicesFb(grpc::ServerBuilder& server_builder)
{
  // create services
  createServicesFb();
  // add services
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "add the flatbuffer gRPC services...";
  server_builder.RegisterService(&*m_metaOperationsFb);
  server_builder.RegisterService(&*m_tfServiceFb);
  server_builder.RegisterService(&*m_imageServiceFb);
}

void server::createServicesPb()
{
  m_metaOperationsPb = std::make_shared<seerep_server::PbMetaOperations>(m_seerepCore);
  m_tfServicePb = std::make_shared<seerep_server::PbTfService>(m_seerepCore);
  m_imageServicePb = std::make_shared<seerep_server::PbImageService>(m_seerepCore);
  m_pointCloudServicePb = std::make_shared<seerep_server::PbPointCloudService>(m_seerepCore);
}

void server::createServicesFb()
{
  m_metaOperationsFb = std::make_shared<seerep_server::FbMetaOperations>(m_seerepCore);
  m_tfServiceFb = std::make_shared<seerep_server::FbTfService>(m_seerepCore);
  m_imageServiceFb = std::make_shared<seerep_server::FbImageService>(m_seerepCore);
}
} /* namespace seerep_server */

void initLogging()
{
  boost::log::add_common_attributes();
  boost::log::add_file_log(
      boost::log::keywords::file_name = "seerep_%N.log", boost::log::keywords::rotation_size = 10 * 1024 * 1024,
      boost::log::keywords::time_based_rotation = boost::log::sinks::file::rotation_at_time_point(0, 0, 0),
      boost::log::keywords::format = "[%TimeStamp%]: %Message%");

  boost::log::add_console_log(std::cout, boost::log::keywords::format = "[%TimeStamp%]: %Message%");

  boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::info);
}

int main(int argc, char** argv)
{
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> logger;
  initLogging();

  std::string datafolder;
  if (argc == 2)
  {
    // use the path from argument
    datafolder = argv[1];
    // append '/' if not path does not end with it
    if (!datafolder.empty() && datafolder.back() != '/')
    {
      datafolder += '/';
    }
  }
  else
  {
    // take the current path if no path is given
    datafolder = std::filesystem::current_path();
  }
  BOOST_LOG_SEV(logger, boost::log::trivial::severity_level::info) << "The used data folder is: ";
  auto seerepCore = std::make_shared<seerep_core::Core>(datafolder);

  // create server builder and set server address / port
  std::string serverAddress = "[::]:9090";
  grpc::ServerBuilder serverBuilder;
  serverBuilder.AddListeningPort(serverAddress, grpc::InsecureServerCredentials());

  // add flatbuffer and protobuf services
  seerep_server::server seerepServer(seerepCore);
  seerepServer.addServicesPb(serverBuilder);
  seerepServer.addServicesFb(serverBuilder);

  // create the server and serve
  std::shared_ptr<grpc::Server> grpcServer = std::shared_ptr<grpc::Server>(serverBuilder.BuildAndStart());
  BOOST_LOG_SEV(logger, boost::log::trivial::severity_level::info)
      << "serving gRPC Server on \"" << serverAddress << "\"...";
  grpcServer->Wait();

  return EXIT_SUCCESS;
}
