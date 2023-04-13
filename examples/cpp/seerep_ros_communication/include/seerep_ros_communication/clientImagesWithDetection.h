#ifndef SEEREP_GRPC_ROS_CLIENT_IMAGES_WITH_DETECTION_H_
#define SEEREP_GRPC_ROS_CLIENT_IMAGES_WITH_DETECTION_H_

#include <functional>
#include <optional>

// grpc
#include <grpc/grpc.h>
#include <grpc/status.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>

// seerep
#include <seerep_com/image_service.grpc.fb.h>
#include <seerep_com/meta_operations.grpc.fb.h>
#include <seerep_com/tf_service.grpc.fb.h>
#include <seerep_ros_conversions_fb/conversions.h>

// ros
#include <ros/master.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_msgs/TFMessage.h>
#include <vision_msgs/Detection2DArray.h>

// pkg
#include "types.h"

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

// geographic lib
#include <GeographicLib/LocalCartesian.hpp>

namespace seerep_grpc_ros
{
class TransferImagesWithDetection
{
public:
  TransferImagesWithDetection(std::shared_ptr<grpc::Channel> channel_ptr, std::string categoryLabels);
  ~TransferImagesWithDetection();

  void send(const sensor_msgs::Image::ConstPtr& msg);

  void send(const vision_msgs::Detection2DArray::ConstPtr& msg);

  void send(const sensor_msgs::NavSatFix::ConstPtr& msg);

private:
  void createSubscriber();
  void createProject();
  // map from pair of seconds/nanoseconds of the header to the uuid
  std::map<uint64_t, std::string> timeUuidMap_;
  std::mutex timeUuidMapMutex_;

  // meta
  std::string projectuuid_;
  StubMetaFbPtr stubMeta_;

  // tf
  StubTfFbPtr stubTf_;
  std::unique_ptr<::grpc::ClientWriter<flatbuffers::grpc::Message<seerep::fb::TransformStamped>>> writerTf_;
  grpc::ClientContext contextTf_;
  flatbuffers::grpc::Message<seerep::fb::ServerResponse> tfResponse_;
  // tf helper
  std::unique_ptr<GeographicLib::LocalCartesian> localCartesian_;

  // image
  StubImageFbPtr stubImage_;
  std::unique_ptr<::grpc::ClientWriter<flatbuffers::grpc::Message<seerep::fb::Image>>> writerImage_;
  grpc::ClientContext contextImage_;
  flatbuffers::grpc::Message<seerep::fb::ServerResponse> imageResponse_;
  // image detections
  std::unique_ptr<::grpc::ClientWriter<flatbuffers::grpc::Message<seerep::fb::BoundingBoxes2DLabeledStamped>>>
      writerImageDetection_;
  grpc::ClientContext contextImageDetection_;
  flatbuffers::grpc::Message<seerep::fb::ServerResponse> imageDetectionResponse_;

  std::string categoryLabels_;

  // ros
  ros::NodeHandle nh_;
  std::map<std::string, ros::Subscriber> subscribers_;
};

} /* namespace seerep_grpc_ros */

#endif  // SEEREP_GRPC_ROS_CLIENT_IMAGES_WITH_DETECTION_H_
