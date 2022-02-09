#include "seerep-server/receive-sensor-msgs.h"

namespace seerep_server
{
ReceiveSensorMsgs::ReceiveSensorMsgs(std::shared_ptr<seerep_core::ProjectOverview> projectOverview)
  : projectOverview(projectOverview)
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
    boost::uuids::uuid uuidImg = projectOverview->addImage(*image, uuid);
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

    projectOverview->addPointCloud(*pointCloud2, uuid);
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
    projectOverview->addTF(*transform, uuid);
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

} /* namespace seerep_server */
