#ifndef SEEREP_GRPC_ROS_ROSBAG_DUMPER
#define SEEREP_GRPC_ROS_ROSBAG_DUMPER

#include <seerep_hdf5_core/hdf5_core_general.h>
#include <seerep_hdf5_core/hdf5_core_image.h>
#include <seerep_hdf5_fb/hdf5_fb_image.h>
#include <seerep_hdf5_fb/hdf5_fb_point.h>
#include <seerep_hdf5_fb/hdf5_fb_tf.h>
#include <seerep_ros_conversions_fb/conversions.h>

#include <filesystem>

// seerep msgs
#include <seerep_msgs/point_stamped_generated.h>

// grpc / flatbuffer
#include <flatbuffers/grpc.h>

// ros
#include <geographic_msgs/GeoPointStamped.h>
#include <ros/console.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_msgs/TFMessage.h>
#include <vision_msgs/Detection2DArray.h>

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
class RosbagDumper
{
public:
  RosbagDumper(std::string bagPath, std::string hdf5FilePath, std::string project_frame_id, std::string project_name,
               std::string topicImage, std::string topicCameraIntrinsics, std::string topicDetection,
               std::string detectionCategory, std::string topicTf, std::string topicTfStatic,
               std::string topicGeoAnchor, float distanceCameraGround);
  ~RosbagDumper();

private:
  void getGeoAnchor();
  void getCameraIntrinsic();
  void iterateAndDumpImages();
  void iterateAndDumpDetections();
  void iterateAndDumpTf();
  void iterateAndDumpTf(const std::string topicTf, const bool isStatic);

  flatbuffers::grpc::Message<seerep::fb::PointStamped> createPointForDetection(vision_msgs::Detection2D detection,
                                                                               int32_t stampSecs, uint32_t stampNanos,
                                                                               std::string frameId, std::string label,
                                                                               std::string instanceUUID);
  void projectPixel(const float u, const float v, const float d, float& X, float& Y, float& Z);
  float calcDiameter(vision_msgs::Detection2D detection);

  std::shared_ptr<seerep_hdf5_core::Hdf5CoreGeneral> m_ioCoreGeneral;
  std::shared_ptr<seerep_hdf5_fb::Hdf5FbTf> m_ioTf;
  std::shared_ptr<seerep_hdf5_fb::Hdf5FbPoint> m_ioPoint;
  std::shared_ptr<seerep_hdf5_fb::Hdf5FbImage> m_ioImage;
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreImage> m_ioImageCore;

  rosbag::Bag bag;
  std::string hdf5FilePath;
  std::string project_frame_id;
  std::string project_name;

  // map from pair of seconds/nanoseconds of the header to the uuid
  std::map<uint64_t, std::string> timeUuidMap_;
  std::mutex timeUuidMapMutex_;

  std::string topicImage;
  std::string topicCameraIntrinsics;
  std::string topicDetection;
  std::string detectionCategory;
  std::string topicTf, topicTfStatic;
  std::string topicGeoAnchor;

  sensor_msgs::CameraInfo cameraInfo;
  float distanceCameraGround;
};

}  // namespace seerep_grpc_ros
#endif  // SEEREP_GRPC_ROS_ROSBAG_DUMPER
