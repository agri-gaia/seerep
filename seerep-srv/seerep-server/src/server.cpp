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
  std::cout << "add the protobuf gRPC services..." << std::endl;
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
  std::cout << "add the flatbuffer gRPC services..." << std::endl;
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

int main(int argc, char** argv)
{
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
  std::cout << "The used data folder is: " << datafolder << std::endl;
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
  std::cout << "serving gRPC Server on \"" << serverAddress << "\"..." << std::endl;
  grpcServer->Wait();

  return EXIT_SUCCESS;
}
