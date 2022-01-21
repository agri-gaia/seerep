#include "seerep-server/server.h"

namespace seerep_server
{
ReceiveSensorMsgs::ReceiveSensorMsgs(std::string datafolder) : projectOverview(datafolder), datafolder(datafolder)
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

  if (!image->header().uuid_project().empty())
  {
    boost::uuids::uuid uuid;
    try
    {
      boost::uuids::string_generator gen;
      uuid = gen(image->header().uuid_project());
    }
    catch (std::runtime_error e)
    {
      // mainly catching "invalid uuid string"
      std::cout << e.what() << std::endl;
      return grpc::Status::CANCELLED;
    }
    boost::uuids::uuid uuidImg = projectOverview.addImage(*image, uuid);
    response->set_message(boost::lexical_cast<std::string>(uuidImg));
    response->set_transmission_state(seerep::ServerResponse::SUCCESS);
    return grpc::Status::OK;
  }
  else
  {
    std::cout << "project_uuid is empty!" << std::endl;
    return grpc::Status::CANCELLED;
  }
}

grpc::Status ReceiveSensorMsgs::GetImage(grpc::ServerContext* context, const seerep::Query* request,
                                         grpc::ServerWriter<seerep::Image>* writer)
{
  std::cout << "sending images in bounding box min(" << request->boundingbox().point_min().x() << "/"
            << request->boundingbox().point_min().y() << "/" << request->boundingbox().point_min().z() << "), max("
            << request->boundingbox().point_max().x() << "/" << request->boundingbox().point_max().y() << "/"
            << request->boundingbox().point_max().z() << ")"
            << " and time interval (" << request->timeinterval().time_min() << "/" << request->timeinterval().time_max()
            << ")" << std::endl;

  std::vector<std::vector<std::optional<seerep::Image>>> images = projectOverview.getImage(*request);
  if (!images.empty())
  {
    std::cout << "Found images in " << images.size() << " projects that match the query" << std::endl;

    for (const std::vector<std::optional<seerep::Image>>& resultPerProject : images)
    {
      std::cout << "Found " << resultPerProject.size() << " images in this projects that match the query" << std::endl;
      for (const std::optional<seerep::Image>& img : resultPerProject)
      {
        writer->Write(img.value());
      }
    }
  }
  else
  {
    std::cout << "Found NOTHING that matches the query" << std::endl;
  }
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferPointCloud2(grpc::ServerContext* context,
                                                    const seerep::PointCloud2* pointCloud2,
                                                    seerep::ServerResponse* response)
{
  std::cout << "received point clouds... " << std::endl;

  if (!pointCloud2->header().uuid_project().empty())
  {
    boost::uuids::uuid uuid;
    try
    {
      boost::uuids::string_generator gen;
      uuid = gen(pointCloud2->header().uuid_project());
    }
    catch (std::runtime_error e)
    {
      // mainly catching "invalid uuid string"
      std::cout << e.what() << std::endl;
      return grpc::Status::CANCELLED;
    }

    projectOverview.addPointCloud(*pointCloud2, uuid);
    response->set_message("okidoki");
    response->set_transmission_state(seerep::ServerResponse::SUCCESS);
    return grpc::Status::OK;
  }
  else
  {
    std::cout << "project_uuid is empty!" << std::endl;
    return grpc::Status::CANCELLED;
  }
}

// grpc::Status ReceiveSensorMsgs::TransferPointCloud2Labeled(grpc::ServerContext* context,
//                                                            const seerep::PointCloud2Labeled* pointCloud2Labeled,
//                                                            seerep::ServerResponse* response)
// {
//   std::cout << "received labeled point clouds... " << std::endl;

//   if (!pointCloud2Labeled->pointcloud().header().uuid_project().empty())
//   {
//     boost::uuids::uuid uuid;
//     try
//     {
//       boost::uuids::string_generator gen;
//       uuid = gen(pointCloud2Labeled->pointcloud().header().uuid_project());
//     }
//     catch (std::runtime_error e)
//     {
//       // mainly catching "invalid uuid string"
//       std::cout << e.what() << std::endl;
//       return grpc::Status::CANCELLED;
//     }
//     projectOverview.addPointCloudLabeled(*pointCloud2Labeled, uuid);
//     response->set_message("okidoki");
//     response->set_transmission_state(seerep::ServerResponse::SUCCESS);
//     return grpc::Status::OK;
//   }
//   else
//   {
//     std::cout << "project_uuid is empty!" << std::endl;
//     return grpc::Status::CANCELLED;
//   }
// }

grpc::Status ReceiveSensorMsgs::GetPointCloud2(grpc::ServerContext* context, const seerep::Query* request,
                                               grpc::ServerWriter<seerep::PointCloud2>* writer)
{
  std::cout << "sending point cloud in bounding box min(" << request->boundingbox().point_min().x() << "/"
            << request->boundingbox().point_min().y() << "/" << request->boundingbox().point_min().z() << "), max("
            << request->boundingbox().point_max().x() << "/" << request->boundingbox().point_max().y() << "/"
            << request->boundingbox().point_max().z() << ")" << std::endl;

  std::vector<std::vector<std::optional<seerep::PointCloud2>>> pointclouds = projectOverview.getPointCloud(*request);
  if (!pointclouds.empty())
  {
    std::cout << "Found pointclouds in " << pointclouds.size() << " projects that match the query" << std::endl;

    for (const std::vector<std::optional<seerep::PointCloud2>>& resultPerProject : pointclouds)
    {
      std::cout << "Found " << resultPerProject.size() << " pointclouds in this projects that match the query"
                << std::endl;
      for (const std::optional<seerep::PointCloud2>& pc : resultPerProject)
      {
        writer->Write(pc.value());
      }
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
  // hdf5_io.writePoint("test_id", *point);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferQuaternion(grpc::ServerContext* context, const seerep::Quaternion* quaternion,
                                                   seerep::ServerResponse* response)
{
  std::cout << "received quaternion... " << std::endl;
  // hdf5_io.writeQuaternion("test_id", *quaternion);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferPose(grpc::ServerContext* context, const seerep::Pose* pose,
                                             seerep::ServerResponse* response)
{
  std::cout << "received pose... " << std::endl;
  // hdf5_io.writePose("test_id", *pose);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferPoseStamped(grpc::ServerContext* context, const seerep::PoseStamped* pose,
                                                    seerep::ServerResponse* response)
{
  std::cout << "received pose_stamped... " << std::endl;
  // hdf5_io.writePoseStamped("test_id", *pose);
  response->set_message("okidoki");
  response->set_transmission_state(seerep::ServerResponse::SUCCESS);
  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::TransferTransformStamped(grpc::ServerContext* context,
                                                         const seerep::TransformStamped* transform,
                                                         seerep::ServerResponse* response)
{
  std::cout << "received transform... " << std::endl;

  if (!transform->header().uuid_project().empty())
  {
    boost::uuids::uuid uuid;
    try
    {
      boost::uuids::string_generator gen;
      uuid = gen(transform->header().uuid_project());
    }
    catch (std::runtime_error e)
    {
      // mainly catching "invalid uuid string"
      std::cout << e.what() << std::endl;
      return grpc::Status::CANCELLED;
    }
    projectOverview.addTF(*transform, uuid);
    response->set_message("added transform");
    response->set_transmission_state(seerep::ServerResponse::SUCCESS);
    return grpc::Status::OK;
  }
  else
  {
    std::cout << "project_uuid is empty!" << std::endl;
    return grpc::Status::CANCELLED;
  }
}

grpc::Status ReceiveSensorMsgs::CreateProject(grpc::ServerContext* context, const seerep::ProjectCreation* request,
                                              seerep::ProjectCreated* response)
{
  std::cout << "create new project... " << std::endl;
  response->set_uuid(projectOverview.newProject(request->name()));

  return grpc::Status::OK;
}

grpc::Status ReceiveSensorMsgs::GetProjects(grpc::ServerContext* context, const google::protobuf::Empty* request,
                                            seerep::ProjectUUIDs* response)
{
  std::cout << "query the project infos... " << std::endl;
  projectOverview.getProjects(response);

  return grpc::Status::OK;
}

std::shared_ptr<grpc::Server> createServer(const std::string& server_address,
                                           seerep_server::ReceiveSensorMsgs* receive_sensor_msgs)
{
  std::cout << "Create the server..." << std::endl;
  grpc::ServerBuilder server_builder;
  server_builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  server_builder.RegisterService(receive_sensor_msgs);
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
  // HighFive::File hdf5_file("test.h5", HighFive::File::ReadWrite | HighFive::File::Create);
  // HighFive::File::ReadWrite | HighFive::File::Create | HighFive::File::Truncate);
  seerep_server::ReceiveSensorMsgs receive_sensor_msgs_service(datafolder);
  std::shared_ptr<grpc::Server> server = seerep_server::createServer(server_address, &receive_sensor_msgs_service);
  std::cout << "serving on \"" << server_address << "\"..." << std::endl;
  server->Wait();
  return EXIT_SUCCESS;
}
