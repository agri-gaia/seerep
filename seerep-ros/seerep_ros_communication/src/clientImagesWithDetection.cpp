#include "seerep_ros_communication/clientImagesWithDetection.h"

namespace seerep_grpc_ros
{
TransferImagesWithDetection::TransferImagesWithDetection(std::shared_ptr<grpc::Channel> channel_ptr)
  : stubImage_(seerep::fb::ImageService::NewStub(channel_ptr))
  , stubMeta_(seerep::fb::MetaOperations::NewStub(channel_ptr))
  , stubTf_(seerep::fb::TfService::NewStub(channel_ptr))
{
  grpc::ClientContext contextTf;
  writerTf_ = stubTf_->TransferTransformStamped(&contextTf, &tfResponse_);

  grpc::ClientContext contextImage;
  writerImage_ = stubImage_->TransferImage(&contextImage, &imageResponse_);

  createProject();
  createSubscriber();
}
TransferImagesWithDetection::~TransferImagesWithDetection()
{
  writerTf_->WritesDone();
  grpc::Status statusTf = writerTf_->Finish();

  std::cout << "tf transfer finish status: " << statusTf.ok() << std::endl;
  std::cout << "tf transfer server response: " << tfResponse_.GetRoot()->message() << std::endl;

  writerImage_->WritesDone();
  grpc::Status statusImage = writerImage_->Finish();

  std::cout << "image transfer finish status: " << statusImage.ok() << std::endl;
  std::cout << "image transfer server response: " << imageResponse_.GetRoot()->message() << std::endl;
}

void seerep_grpc_ros::TransferImagesWithDetection::send(const sensor_msgs::Image::ConstPtr& msg)
{
  boost::uuids::uuid msguuid = boost::uuids::random_generator()();
  std::string uuidstring = boost::lexical_cast<std::string>(msguuid);
  if (writerImage_->Write(seerep_ros_conversions_fb::toFlat(*msg, projectuuid_, uuidstring)))
  {
    uint64_t time = (uint64_t)msg->header.stamp.sec << 32 | msg->header.stamp.nsec;
    // auto thepair = std::make_pair(time, uuidstring);
    {  // scope of lock
      const std::scoped_lock lock(timeUuidMapMutex_);
      timeUuidMap_.emplace(time, uuidstring);
    }
  }
  else
  {
    ROS_ERROR_STREAM("error while transfering image");
  }
}

void seerep_grpc_ros::TransferImagesWithDetection::send(const vision_msgs::Detection2DArray::ConstPtr& msg)
{
  uint64_t time = (uint64_t)msg->header.stamp.sec << 32 | msg->header.stamp.nsec;
  std::string uuidstring;
  int i = 0;

  for (int i = 0; i < 10; i++)
  {
    {  // scope of lock
      const std::scoped_lock lock(timeUuidMapMutex_);
      auto result = timeUuidMap_.find(time);
      if (result != timeUuidMap_.end())
      {
        uuidstring = result->second;
        break;
      }
    }
  }
  auto fbmsg = seerep_ros_conversions_fb::toFlat(*msg, projectuuid_, uuidstring);
  // if (writerImage_->Write(seerep_ros_conversions_fb::toFlat(*msg, projectuuid_, uuidstring))) {}
  // else
  // {
  //   ROS_ERROR_STREAM("error while transfering image");
  // }
}

// void seerep_grpc_ros::TransferImagesWithDetection::send(const tf2_msgs::TFMessage::ConstPtr& msg) const
// {
//   grpc::ClientContext context;
//   flatbuffers::grpc::Message<seerep::fb::ServerResponse> response;
//   for (auto tf : msg->transforms)
//   {
//     grpc::Status status =
//         stubTf_->TransferTransformStamped(&context, seerep_ros_conversions_fb::toProto(tf, projectuuid), &response);
//     if (!status.ok())
//     {
//       ROS_ERROR_STREAM("gRPC status error code: " << status.error_code() << " " << status.error_message());
//     }
//     else
//     {
//       ROS_INFO_STREAM("Response:" << response.message());
//     }
//   }
// }

void TransferImagesWithDetection::createSubscriber()
{
  nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 0, &TransferImagesWithDetection::send, this);
  nh.subscribe<vision_msgs::Detection2DArray>("/camera/color/Detection2DArray", 0, &TransferImagesWithDetection::send,
                                              this);
  // nh.subscribe<tf2_msgs::TFMessage>("/tf", 0, &TransferImagesWithDetection::send, this);
}

void TransferImagesWithDetection::createProject()
{
  flatbuffers::grpc::MessageBuilder mb;
  // create the request msg
  seerep::fb::ProjectCreationBuilder builder(mb);
  builder.add_map_frame_id(mb.CreateString("map"));
  builder.add_name(mb.CreateString("agricultural_demonstator"));
  auto request_offset = builder.Finish();
  mb.Finish(request_offset);
  auto request_msg = mb.ReleaseMessage<seerep::fb::ProjectCreation>();

  // call the server
  grpc::ClientContext contextMeta;
  flatbuffers::grpc::Message<seerep::fb::ProjectInfo> response_msg;
  auto status = stubMeta_->CreateProject(&contextMeta, request_msg, &response_msg);

  // store answer
  projectuuid_ = response_msg.GetRoot()->uuid()->str();
  ROS_INFO_STREAM("Project created: " << response_msg.GetRoot()->name()->str() << " " << projectuuid_);
}
}  // namespace seerep_grpc_ros

/* namespace seerep_grpc_ros */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "seerep_ros_communication_client_transferImagesWithDetection");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::map<std::string, ros::Subscriber> subscribers;
  std::vector<std::string> topics;

  std::string server_address;
  private_nh.param<std::string>("server_address", server_address, "localhost:9090");

  seerep_grpc_ros::TransferImagesWithDetection transferImagesWithDetection(
      grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()));

  ros::spin();

  return EXIT_SUCCESS;
}
