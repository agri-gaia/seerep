#include "seerep-server/server.h"

namespace seerep_server
{
ReceiveSensorMsgs::ReceiveSensorMsgs(HighFive::File& file) : hdf5_io(file), pcOverview("testProject")
{
}

grpc::Status ReceiveSensorMsgs::TransferHeader(grpc::ServerContext* context, const seerep::Header* header,
                                               seerep::ServerResponse* response)
{
  std::cout << "received header... " << std::endl;
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferImage(grpc::ServerContext* context, const seerep::Image* image,
                                              seerep::ServerResponse* response)
{
  std::cout << "received image... " << std::endl;
  hdf5_io.writeImage("test_id", *image);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferPointCloud2(grpc::ServerContext* context,
                                                    const seerep::PointCloud2* point_cloud_2,
                                                    seerep::ServerResponse* response)
{
  std::cout << "received point clouds... " << std::endl;
  // TODO implement hdf5_io function
  // hdf5_io.writePointCloud2("test_id", *point_cloud_2);
  pcOverview.addDataset("test_id", *point_cloud_2);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::GetPointCloud2(grpc::ServerContext* context, const seerep::Boundingbox* request,
                                               grpc::ServerWriter<seerep::PointCloud2>* writer)
{
  std::cout << "sending point cloud in bounding box min(" << request->point_min().x() << "/" << request->point_min().y()
            << "/" << request->point_min().z() << "), max(" << request->point_max().x() << "/"
            << request->point_max().y() << "/" << request->point_max().z() << ")" << std::endl;
  // TODO implement hdf5_io function
  std::vector<std::unique_ptr<seerep::PointCloud2>> pointclouds = pcOverview.getData("test_id", *request);
  if (!pointclouds.empty())
  {
    std::cout << "Found " << pointclouds.size() + "pointclouds that match the query" << std::endl;

    for (const std::unique_ptr<seerep::PointCloud2>& pc : pointclouds)
    {
      writer->Write(*pc.get());
    }
  }
  else
  {
    std::cout << "Found NOTHING that matches the query" << std::endl;
  }
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferPoint(grpc::ServerContext* context, const seerep::Point* point,
                                              seerep::ServerResponse* response)
{
  std::cout << "received point... " << std::endl;
  hdf5_io.writePoint("test_id", *point);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferQuaternion(grpc::ServerContext* context, const seerep::Quaternion* quaternion,
                                                   seerep::ServerResponse* response)
{
  std::cout << "received quaternion... " << std::endl;
  hdf5_io.writeQuaternion("test_id", *quaternion);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferPose(grpc::ServerContext* context, const seerep::Pose* pose,
                                             seerep::ServerResponse* response)
{
  std::cout << "received pose... " << std::endl;
  hdf5_io.writePose("test_id", *pose);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferPoseStamped(grpc::ServerContext* context, const seerep::PoseStamped* pose,
                                                    seerep::ServerResponse* response)
{
  std::cout << "received pose_stamped... " << std::endl;
  hdf5_io.writePoseStamped("test_id", *pose);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

std::shared_ptr<grpc::Server> createServer(const std::string& server_address,
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
  HighFive::File hdf5_file("test.h5", HighFive::File::ReadWrite | HighFive::File::Create);
  // HighFive::File::ReadWrite | HighFive::File::Create | HighFive::File::Truncate);
  seerep_server::ReceiveSensorMsgs receive_sensor_msgs_service(hdf5_file);
  std::shared_ptr<grpc::Server> server = seerep_server::createServer(server_address, &receive_sensor_msgs_service);
  std::cout << "serving on \"" << server_address << "\"..." << std::endl;
  server->Wait();
  return EXIT_SUCCESS;
}
