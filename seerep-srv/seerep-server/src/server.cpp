#include "seerep-server/server.h"

namespace seerep_server
{
std::shared_ptr<grpc::Server> createServer(const std::string& server_address,
                                           seerep_server::MetaOperations* metaOperations,
                                           seerep_server::QueryData* queryData,
                                           seerep_server::ReceiveSensorMsgs* receiveSensorMsgs)
{
  std::cout << "Create the server..." << std::endl;
  grpc::ServerBuilder server_builder;
  server_builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  server_builder.RegisterService(metaOperations);
  server_builder.RegisterService(queryData);
  server_builder.RegisterService(receiveSensorMsgs);
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

  auto projectOverview = std::make_shared<seerep_core::ProjectOverview>(datafolder);

  seerep_server::MetaOperations metaOperationsService(projectOverview);
  seerep_server::QueryData queryDataService(projectOverview);
  seerep_server::ReceiveSensorMsgs receiveSensorMsgsService(projectOverview);
  std::shared_ptr<grpc::Server> server =
      seerep_server::createServer(server_address, &metaOperationsService, &queryDataService, &receiveSensorMsgsService);
  std::cout << "serving on \"" << server_address << "\"..." << std::endl;
  server->Wait();
  return EXIT_SUCCESS;
}
