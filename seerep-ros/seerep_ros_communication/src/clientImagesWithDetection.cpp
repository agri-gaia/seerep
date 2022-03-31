#include "seerep_ros_communication/clientImagesWithDetection.h"

namespace seerep_grpc_ros
{
TransferImagesWithDetection::TransferImagesWithDetection(std::shared_ptr<grpc::Channel> channel_ptr)
  : stubImage_(seerep::fb::ImageService::NewStub(channel_ptr))
  , stubMeta_(seerep::fb::MetaOperations::NewStub(channel_ptr))
  , stubTf_(seerep::fb::TfService::NewStub(channel_ptr))
{
  writerTf_ = stubTf_->TransferTransformStamped(&contextTf_, &tfResponse_);

  writerImage_ = stubImage_->TransferImage(&contextImage_, &imageResponse_);

  writerImageDetection_ = stubImage_->AddBoundingBoxes2dLabeled(&contextImageDetection_, &imageDetectionResponse_);

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

  std::cout << "transfering image to server " << uuidstring << std::endl;

  if (writerImage_->Write(seerep_ros_conversions_fb::toFlat(*msg, projectuuid_, uuidstring)))
  {
    uint64_t time = (uint64_t)msg->header.stamp.sec << 32 | msg->header.stamp.nsec;

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

  // try for 10 secs
  // for (int i = 0; i < 10; i++)
  // {
  {  // scope of lock
    const std::scoped_lock lock(timeUuidMapMutex_);
    auto result = timeUuidMap_.find(time);
    if (result != timeUuidMap_.end())
    {
      uuidstring = result->second;
      // break;
    }
  }
  //   ros::Duration(1).sleep();
  // }

  std::cout << "transfering detections to server: " << uuidstring << std::endl;

  if (uuidstring.empty())
  {
    ROS_ERROR_STREAM("uuid of detection is empty");
    std::cout << "time: " << msg->header.stamp.sec << " / " << msg->header.stamp.nsec << std::endl;
  }
  else
  {
    if (writerImageDetection_->Write(seerep_ros_conversions_fb::toFlat(*msg, projectuuid_, uuidstring))) {}
    else
    {
      ROS_ERROR_STREAM("error while transfering detection");
    }
  }
}

void seerep_grpc_ros::TransferImagesWithDetection::send(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  std::cout << "transfering tf to server" << msg->header.stamp.sec << std::endl;
  if (!localCartesian_)
  {
    localCartesian_ =
        std::move(std::make_unique<GeographicLib::LocalCartesian>(msg->latitude, msg->longitude, msg->altitude));
  }
  double x, y, z;
  localCartesian_->Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);
  geometry_msgs::TransformStamped transform;
  transform.header = msg->header;
  transform.header.frame_id = "map";
  transform.child_frame_id = "camera_color_optical_frame ";
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;
  transform.transform.rotation.x = 0;
  transform.transform.rotation.y = 0;
  transform.transform.rotation.z = 0;
  transform.transform.rotation.w = 1;

  if (!writerTf_->Write(seerep_ros_conversions_fb::toFlat(transform, projectuuid_)))
  {
    ROS_ERROR_STREAM("error while transfering tf");
  }
}

void TransferImagesWithDetection::createSubscriber()
{
  subscribers_.emplace("/camera/color/image_raw",
                       nh_.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 10,
                                                         &TransferImagesWithDetection::send, this));
  subscribers_.emplace("/camera/color/Detection2DArray",
                       nh_.subscribe<vision_msgs::Detection2DArray>("/camera/color/Detection2DArray", 10,
                                                                    &TransferImagesWithDetection::send, this));
  subscribers_.emplace("/fix",
                       nh_.subscribe<sensor_msgs::NavSatFix>("/fix", 10, &TransferImagesWithDetection::send, this));
}

void TransferImagesWithDetection::createProject()
{
  flatbuffers::grpc::MessageBuilder mb;

  auto frameIdOffset = mb.CreateString("map");
  auto nameOffset = mb.CreateString("agricultural_demonstator");
  // create the request msg
  seerep::fb::ProjectCreationBuilder builder(mb);
  builder.add_map_frame_id(frameIdOffset);
  builder.add_name(nameOffset);
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
  ros::NodeHandle private_nh("~");

  std::string server_address;
  private_nh.param<std::string>("server_address", server_address, "localhost:9090");

  seerep_grpc_ros::TransferImagesWithDetection transferImagesWithDetection(
      grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()));

  // ros::spin();
  auto spinner = ros::MultiThreadedSpinner(4);
  spinner.spin();

  return EXIT_SUCCESS;
}
