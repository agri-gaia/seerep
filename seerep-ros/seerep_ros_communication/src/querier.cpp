#include "seerep_ros_communication/querier.h"
#include "seerep_ros_communication/types.h"
#include <grpc/status.h>

namespace seerep_grpc_ros
{
QueryData::QueryData(std::shared_ptr<grpc::Channel> channel_ptr)
  : stub_(seerep::TransferSensorMsgs::NewStub(channel_ptr))
{
}

void QueryData::queryPointcloud(const seerep::Boundingbox& bb, ros::Publisher& pc2_pub) const
{
  grpc::ClientContext context;
  seerep::PointCloud2 response;
  std::unique_ptr<grpc::ClientReader<seerep::PointCloud2>> reader = stub_->GetPointCloud2(&context, bb);

  while (reader->Read(&response))
  {
    sensor_msgs::PointCloud2 pc2 = seerep_ros_conversions::toROS(response);
    pc2.header.frame_id = "map";

    ROS_INFO_STREAM("publish pointcloud" << pc2);
    pc2_pub.publish(pc2);

    ros::spinOnce();
  }
  grpc::Status status = reader->Finish();
}

} /* namespace seerep_grpc_ros */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "seerep_ros_communication_client");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::Publisher pc2_pub = nh.advertise<sensor_msgs::PointCloud2>("queried_pc", 1000);

  std::string server_address;
  private_nh.param<std::string>("server_address", server_address, "localhost:9090");

  seerep_grpc_ros::QueryData query_data(grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()));

  seerep::Boundingbox bb;
  bb.mutable_point_min()->set_x(0);
  bb.mutable_point_min()->set_y(0);
  bb.mutable_point_min()->set_z(0);
  bb.mutable_point_max()->set_x(10);
  bb.mutable_point_max()->set_y(10);
  bb.mutable_point_max()->set_z(10);

  ROS_INFO("Topic is published. Connect to it now. Press enter to resume.");
  std::cin.get();
  // ros::Duration(10.0).sleep();
  query_data.queryPointcloud(bb, pc2_pub);

  return EXIT_SUCCESS;
}
