#ifndef SEEREP_ROS_COMM_ROSBAG_DUMPER_H_
#define SEEREP_ROS_COMM_ROSBAG_DUMPER_H_

#include <curl/curl.h>
#include <jsoncpp/json/json.h>
#include <seerep_core_fb/core_fb_conversion.h>
#include <seerep_hdf5_core/hdf5_core_cameraintrinsics.h>
#include <seerep_hdf5_core/hdf5_core_general.h>
#include <seerep_hdf5_core/hdf5_core_image.h>
#include <seerep_hdf5_fb/hdf5_fb_image.h>
#include <seerep_hdf5_fb/hdf5_fb_point.h>
#include <seerep_hdf5_fb/hdf5_fb_tf.h>
#include <seerep_ros_conversions_fb/conversions.h>

#include <filesystem>
#include <fstream>

// seerep msgs
#include <seerep_msgs/point_stamped_generated.h>

// grpc / flatbuffer
#include <flatbuffers/grpc.h>

// ros
#include <cv_bridge/cv_bridge.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <ros/console.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_msgs/TFMessage.h>
#include <vision_msgs/Detection2DArray.h>

#include <opencv2/opencv.hpp>

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

// geographic lib
#include <GeographicLib/LocalCartesian.hpp>

namespace seerep_ros_comm
{
class RosbagDumper
{
public:
  RosbagDumper(const std::string& bagPath,
               const std::string& classesMappingPath,
               const std::string& hdf5FilePath,
               const std::string& projectFrameId,
               const std::string& projectName, const std::string& projectUuid,
               const std::string& topicImage,
               const std::string& topicCameraIntrinsics,
               const std::string& topicDetection, const std::string& topicTf,
               const std::string& topicTfStatic,
               const std::string& topicGeoAnchor, float distanceCameraGround,
               double maxViewingDistance, bool storeImages = true);
  ~RosbagDumper();

private:
  void readClassesMapping(const std::string& classesMappingPath);
  void getGeoAnchor();
  void getCameraIntrinsic();
  void iterateAndDumpImages();
  void iterateAndDumpImagesCompressed();
  void iterateAndDumpDetections(bool storeImages);
  void iterateAndDumpTf();
  void iterateAndDumpTf(const std::string& topicTf, const bool isStatic);
  sensor_msgs::Image::ConstPtr convertCompressedImageToImage(
      const sensor_msgs::CompressedImage::ConstPtr& msg);

  std::string translateNameToAgrovocConcept(std::string name);
  std::unordered_map<std::string, std::string> name2Concept;

  flatbuffers::grpc::Message<seerep::fb::PointStamped>
  createPointForDetection(vision_msgs::Detection2D detection, int32_t stampSecs,
                          uint32_t stampNanos, const std::string& frameId,
                          const std::string& labelAgrovoc,
                          const std::string& labelTrivial,
                          const std::string& instanceUUID);
  void projectPixel(const float u, const float v, const float d, float& X,
                    float& Y, float& Z);
  float calcDiameter(vision_msgs::Detection2D detection);

  std::shared_ptr<seerep_hdf5_core::Hdf5CoreGeneral> ioCoreGeneral;
  std::shared_ptr<seerep_hdf5_fb::Hdf5FbTf> ioTf;
  std::shared_ptr<seerep_hdf5_fb::Hdf5FbPoint> ioPoint;
  std::shared_ptr<seerep_hdf5_fb::Hdf5FbImage> ioImage;
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreImage> ioImageCore;
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreCameraIntrinsics> ioCameraIntrinsics;

  std::unordered_map<int64_t, std::string> classesMapping;

  rosbag::Bag bag;
  std::string hdf5FilePath;
  std::string projectFrameId;
  std::string projectName;
  std::string projectUuid;

  // map from pair of seconds/nanoseconds of the header to the uuid
  std::map<uint64_t, std::string> timeUuidMap;

  std::string topicImage;
  std::string topicCameraIntrinsics;
  std::string topicDetection;
  std::string topicTf, topicTfStatic;
  std::string topicGeoAnchor;

  sensor_msgs::CameraInfo cameraInfo;
  std::string cameraIntrinsicsUuid;
  float distanceCameraGround;
  double maxViewingDistance;
};

}  // namespace seerep_ros_comm

#endif  // SEEREP_ROS_COMM_ROSBAG_DUMPER_H_
