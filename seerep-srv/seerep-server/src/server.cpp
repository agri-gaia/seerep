#include "seerep-server/server.h"

namespace seerep_server
{
std::shared_ptr<grpc::Server> createServerPb(const std::string& server_address,
                                             seerep_server::PbMetaOperations* metaOperations,
                                             seerep_server::PbTfService* tfService,
                                             seerep_server::PbImageService* imageService,
                                             seerep_server::PbPointCloudService* pointCloudService)
// seerep_server::ReceiveSensorMsgs* receiveSensorMsgs,
{
  std::cout << "Create the server..." << std::endl;
  grpc::ServerBuilder server_builder;
  server_builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  server_builder.RegisterService(metaOperations);
  server_builder.RegisterService(tfService);
  server_builder.RegisterService(imageService);
  server_builder.RegisterService(pointCloudService);
  // server_builder.RegisterService(receiveSensorMsgs);
  return std::shared_ptr<grpc::Server>(server_builder.BuildAndStart());
}
std::shared_ptr<grpc::Server> createServerFb(const std::string& server_address,
                                             seerep_server::FbMetaOperations* metaOperations)
// seerep_server::ReceiveSensorMsgs* receiveSensorMsgs,
{
  std::cout << "Create the server..." << std::endl;
  grpc::ServerBuilder server_builder;
  server_builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  server_builder.RegisterService(metaOperations);
  return std::shared_ptr<grpc::Server>(server_builder.BuildAndStart());
}
} /* namespace seerep_server */

int main(int argc, char** argv)
{
  std::string datafolder;
  if (argc == 2)
  {
    datafolder = argv[1];
    // append '/' if not path does not end with it
    if (!datafolder.empty() && datafolder.back() != '/')
    {
      datafolder += '/';
    }
  }
  else
  {
    datafolder = std::filesystem::current_path();
  }
  std::cout << "The used data folder is: " << datafolder << std::endl;
  std::string server_address = "0.0.0.0:9090";

  auto seerepCore = std::make_shared<seerep_core::Core>(datafolder);

  if (false)
  {
    seerep_server::PbMetaOperations metaOperationsService(seerepCore);
    seerep_server::PbTfService tfService(seerepCore);
    seerep_server::PbImageService imageService(seerepCore);
    seerep_server::PbPointCloudService pointCloudService(seerepCore);
    // seerep_server::ReceiveSensorMsgs receiveSensorMsgsService(seerepCore);
    std::shared_ptr<grpc::Server> server =
        seerep_server::createServerPb(server_address, &metaOperationsService, &tfService, &imageService,
                                      &pointCloudService);  //&receiveSensorMsgsService, ,
    std::cout << "serving on \"" << server_address << "\"..." << std::endl;
    server->Wait();
  }
  else
  {
    seerep_server::FbMetaOperations metaOperationsService(seerepCore);
    std::shared_ptr<grpc::Server> server = seerep_server::createServerFb(server_address, &metaOperationsService);
    std::cout << "serving on \"" << server_address << "\"..." << std::endl;
    server->Wait();
  }
  return EXIT_SUCCESS;
}
