#ifndef SEEREP_GRPC_ROS_SERVER_H_
#define SEEREP_GRPC_ROS_SERVER_H_

// grpc
#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/security/server_credentials.h>

// ag
#include <seerep-com/transfer_sensor_msgs.grpc.pb.h>
#include <seerep_ros_conversions/conversions.h>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace seerep_grpc_ros
{
class ReceiveSensorMsgs final : public seerep::TransferSensorMsgs::Service
{
public:
  ReceiveSensorMsgs();
  grpc::Status TransferPointCloud2(grpc::ServerContext* context, const seerep::PointCloud2* point_cloud_2, seerep::ServerResponse* response);
  grpc::Status TransferImage(grpc::ServerContext* context, const seerep::Image* image, seerep::ServerResponse* response);
  grpc::Status TransferHeader(grpc::ServerContext* context, const seerep::Header* header, seerep::ServerResponse* response);
  grpc::Status TransferPoint(grpc::ServerContext* context, const seerep::Point* point , seerep::ServerResponse* response);
  grpc::Status TransferQuaternion(grpc::ServerContext* context, const seerep::Quaternion* quaternion, seerep::ServerResponse* response);
  grpc::Status TransferPose(grpc::ServerContext* context, const seerep::Pose* pose, seerep::ServerResponse* response);
  grpc::Status TransferPoseStamped(grpc::ServerContext* context, const seerep::PoseStamped* pose, seerep::ServerResponse* response);
};

std::shared_ptr<grpc::Server> createServer(const std::string& server_address, seerep_grpc_ros::ReceiveSensorMsgs* receive_sensor_msgs);

} /* namespace seerep_grpc_ros */

#endif // SEEREP_GRPC_ROS_SERVER_H_
