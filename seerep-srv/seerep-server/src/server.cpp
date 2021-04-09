#include "seerep-server/server.h"

namespace seerep_server
{
ReceiveSensorMsgs::ReceiveSensorMsgs(HighFive::File &file)
  : hdf5_io(file) {}

grpc::Status ReceiveSensorMsgs::TransferHeader(
    grpc::ServerContext* context,
    const seerep::Header* header,
    seerep::ServerResponse* response)
{
  std::cout << "received header... " << std::endl;
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferImage(
    grpc::ServerContext* context,
    const seerep::Image* image,
    seerep::ServerResponse* response)
{
  std::cout << "received image... " << std::endl;
  hdf5_io.writeImage  ("test_id", *image);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferPointCloud2(
    grpc::ServerContext* context,
    const seerep::PointCloud2* point_cloud_2,
    seerep::ServerResponse* response)
{
  std::cout << "received point clouds... " << std::endl;
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

std::shared_ptr<grpc::Server> createServer(
    const std::string& server_address,
    seerep_server::ReceiveSensorMsgs* receive_sensor_msgs)
{
  grpc::ServerBuilder server_builder;
  server_builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  server_builder.RegisterService(receive_sensor_msgs);
  return std::shared_ptr<grpc::Server>(server_builder.BuildAndStart());
}

} /* namespace seerep_server */


int main(int argc, char** argv)
{

  std::string server_address = "localhost:9090";
  HighFive::File hdf5_file("test.h5", HighFive::File::ReadWrite | HighFive::File::Create | HighFive::File::Truncate);
  seerep_server::ReceiveSensorMsgs receive_sensor_msgs_service(hdf5_file);
  std::shared_ptr<grpc::Server> server = seerep_server::createServer(server_address, &receive_sensor_msgs_service);
  std::cout << "serving on \"" << server_address << "\"..." << std::endl;
  server->Wait();
  return EXIT_SUCCESS;
}
