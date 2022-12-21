#include "seerep_ros_communication/querier.h"

namespace seerep_grpc_ros
{
QueryData::QueryData(std::shared_ptr<grpc::Channel> channel_ptr)
  : stubImage_(seerep::ImageService::NewStub(channel_ptr))
  , stubPointCloud_(seerep::PointCloudService::NewStub(channel_ptr))
{
}

void QueryData::queryPointcloud(const seerep::Query& query, ros::Publisher& pc2_pub) const
{
  grpc::ClientContext context;
  seerep::PointCloud2 response;
  std::unique_ptr<grpc::ClientReader<seerep::PointCloud2>> reader = stubPointCloud_->GetPointCloud2(&context, query);

  while (reader->Read(&response))
  {
    sensor_msgs::PointCloud2 pc2 = seerep_ros_conversions_pb::toROS(response);
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
  std::unique_ptr<grpc::ClientReader<seerep::Image>> reader = stubImage_->GetImage(&context, query);

  while (reader->Read(&response))
  {
    sensor_msgs::Image img = seerep_ros_conversions_pb::toROS(response);
    img.header.frame_id = "map";

    for (auto bbCat : response.labeled_bounding_boxes())
    {
      std::cout << "category: " << bbCat.category() << std::endl;
      for (auto bb : bbCat.labeled_2d_bounding_boxes())
      {
        std::cout << "label: " << bb.label_with_instance().label() << " box: " << bb.bounding_box().point_min().x()
                  << " / " << bb.bounding_box().point_min().y() << " / " << bb.bounding_box().point_max().x() << " / "
                  << bb.bounding_box().point_max().y() << std::endl;
      }
    }

    for (auto labelsCat : response.general_labels())
    {
      std::cout << "category: " << labelsCat.category() << std::endl;
      for (auto label : labelsCat.label_with_instances())
      {
        std::cout << "label_general: " << label.label() << std::endl;
      }
    }

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

  // grpc server
  std::string server_address;
  private_nh.param<std::string>("server_address", server_address, "localhost:9090");
  seerep_grpc_ros::QueryData query_data(grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()));

  // publish topic PC
  std::string topicqueriedpc;
  private_nh.param<std::string>("topicqueriedpc", topicqueriedpc, "queried_pc");
  ros::Publisher pc2_pub = nh.advertise<sensor_msgs::PointCloud2>(topicqueriedpc, 1000);

  // publish topic img
  std::string topicqueriedimg;
  private_nh.param<std::string>("topicqueriedimg", topicqueriedimg, "queried_img");
  ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>(topicqueriedimg, 1000);

  seerep::Query query;

  // spatial
  double minx, miny, minz, maxx, maxy, maxz;
  if (private_nh.param<double>("point_min_x", minx, 0.0) && private_nh.param<double>("point_min_y", miny, 0.0) &&
      private_nh.param<double>("point_min_z", minz, 0.0) && private_nh.param<double>("point_max_x", maxx, 0.0) &&
      private_nh.param<double>("point_max_y", maxy, 0.0) && private_nh.param<double>("point_max_z", maxz, 0.0))
  {
    query.mutable_bounding_box_stamped()->mutable_bounding_box()->mutable_point_min()->set_x(minx);
    query.mutable_bounding_box_stamped()->mutable_bounding_box()->mutable_point_min()->set_y(miny);
    query.mutable_bounding_box_stamped()->mutable_bounding_box()->mutable_point_min()->set_z(minz);
    query.mutable_bounding_box_stamped()->mutable_bounding_box()->mutable_point_max()->set_x(maxx);
    query.mutable_bounding_box_stamped()->mutable_bounding_box()->mutable_point_max()->set_y(maxy);
    query.mutable_bounding_box_stamped()->mutable_bounding_box()->mutable_point_max()->set_z(maxz);
  }
  // temporal
  int mintime, maxtime;
  if (private_nh.param<int>("time_min", mintime, 0) && private_nh.param<int>("time_max", maxtime, 0))
  {
    query.mutable_time_interval()->mutable_time_min()->set_seconds(mintime);
    query.mutable_time_interval()->mutable_time_max()->set_seconds(maxtime);
  }
  // semantic
  std::string category;
  std::vector<std::string> labels;
  if (private_nh.param<std::vector<std::string>>("labels", labels, std::vector<std::string>()) &&
      private_nh.param<std::string>("category", category, std::string()))
  {
    auto labelWithCategory = query.add_categorized_labels();
    labelWithCategory->set_category(category);
    for (auto label : labels)
    {
      labelWithCategory->add_labels(label);
    }
  }

  bool musthavealllabels;
  if (private_nh.param<bool>("musthavealllabels", musthavealllabels, false))
  {
    query.set_must_have_all_labels(musthavealllabels);
  }

  int maxNumData;
  if (private_nh.param<int>("maxNumData", maxNumData, 0))
  {
    query.set_max_num_data(maxNumData);
  }

  ROS_INFO("Topic is published. Connect to it now. Press enter to resume.");
  std::cin.get();

  // query and publish data
  query_data.queryImage(query, img_pub);

  return EXIT_SUCCESS;
}
