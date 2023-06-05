#ifndef SEEREP_GRPC_ROS_JSON_POINT_DUMPER
#define SEEREP_GRPC_ROS_JSON_POINT_DUMPER

#include <jsoncpp/json/json.h>
#include <ros/ros.h>
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
class JsonPointDumper
{
public:
  JsonPointDumper(const std::string& filePath, const std::string& hdf5FilePath, const std::string& detectionCategory);
  ~JsonPointDumper();

private:
  void readAndDumpJson(const std::string& jsonFilePath);
  flatbuffers::grpc::Message<seerep::fb::PointStamped>
  createPointForDetection(int32_t stampSecs, uint32_t stampNanos, const std::string& frameId, const std::string& label,
                          const std::string& instanceUUID, const float x, const float y, const float z,
                          const float diameter);

  std::shared_ptr<seerep_hdf5_core::Hdf5CoreGeneral> ioCoreGeneral;
  std::shared_ptr<seerep_hdf5_fb::Hdf5FbPoint> ioPoint;

  std::string detectionCategory;
};

}  // namespace seerep_grpc_ros
#endif  // SEEREP_GRPC_ROS_JSON_POINT_DUMPER
