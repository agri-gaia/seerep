#include "seerep_ros_communication/querier.h"

namespace seerep_grpc_ros
{
QueryData::QueryData(std::shared_ptr<grpc::Channel> channel_ptr)
  : stub_(seerep::TransferSensorMsgs::NewStub(channel_ptr))
{
}

void QueryData::queryPointcloud(const seerep::Query& query, ros::Publisher& pc2_pub) const
{
  grpc::ClientContext context;
  seerep::PointCloud2 response;
  std::unique_ptr<grpc::ClientReader<seerep::PointCloud2>> reader = stub_->GetPointCloud2(&context, query);

  while (reader->Read(&response))
  {
    sensor_msgs::PointCloud2 pc2 = seerep_ros_conversions::toROS(response);
    pc2.header.frame_id = "map";

    ROS_INFO_STREAM("publish pointcloud\n" << pc2);
    pc2_pub.publish(pc2);

    ros::spinOnce();
  }
  grpc::Status status = reader->Finish();
}

void QueryData::queryImage(const seerep::Query& query, ros::Publisher& img_pub) const
{
  grpc::ClientContext context;
  seerep::Image response;
  std::unique_ptr<grpc::ClientReader<seerep::Image>> reader = stub_->GetImage(&context, query);

  while (reader->Read(&response))
  {
    sensor_msgs::Image img = seerep_ros_conversions::toROS(response);
    img.header.frame_id = "map";

    ROS_INFO_STREAM("publish image\n" << img);
    img_pub.publish(img);

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
  ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("queried_img", 1000);

  std::string server_address;
  private_nh.param<std::string>("server_address", server_address, "localhost:9090");

  seerep_grpc_ros::QueryData query_data(grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()));

  seerep::Query query;
  query.mutable_boundingbox()->mutable_point_min()->set_x(0);
  query.mutable_boundingbox()->mutable_point_min()->set_y(0);
  query.mutable_boundingbox()->mutable_point_min()->set_z(0);
  query.mutable_boundingbox()->mutable_point_max()->set_x(10);
  query.mutable_boundingbox()->mutable_point_max()->set_y(10);
  query.mutable_boundingbox()->mutable_point_max()->set_z(10);

  query.mutable_timeinterval()->set_time_min(1638549273);
  query.mutable_timeinterval()->set_time_max(1638549276);

  ROS_INFO("Topic is published. Connect to it now. Press enter to resume.");
  std::cin.get();
  // ros::Duration(10.0).sleep();
  // query_data.queryPointcloud(query, pc2_pub);
  query_data.queryImage(query, img_pub);

  return EXIT_SUCCESS;
}
