#include "seerep-server/server.h"

namespace seerep_server
{
std::shared_ptr<grpc::Server> createServer(const std::string& server_address,
                                           seerep_server::MetaOperations* metaOperations,
                                           seerep_server::ImageService* imageService,
                                           seerep_server::TfService* tfService)
// seerep_server::ReceiveSensorMsgs* receiveSensorMsgs, seerep_server::PointCloudService* pointCloudService,
{
  std::cout << "Create the server..." << std::endl;
  grpc::ServerBuilder server_builder;
  server_builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  server_builder.RegisterService(metaOperations);
  // server_builder.RegisterService(receiveSensorMsgs);
  server_builder.RegisterService(imageService);
  // server_builder.RegisterService(pointCloudService);
  server_builder.RegisterService(tfService);
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

  seerep_server::MetaOperations metaOperationsService(seerepCore);
  // seerep_server::ReceiveSensorMsgs receiveSensorMsgsService(seerepCore);
  seerep_server::ImageService imageService(seerepCore);
  // seerep_server::PointCloudService pointCloudService(seerepCore);
  seerep_server::TfService tfService(seerepCore);
  std::shared_ptr<grpc::Server> server =
      seerep_server::createServer(server_address, &metaOperationsService, &imageService,
                                  &tfService);  //&receiveSensorMsgsService, &pointCloudService,
  std::cout << "serving on \"" << server_address << "\"..." << std::endl;
  server->Wait();
  return EXIT_SUCCESS;
}
