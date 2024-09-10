#ifndef SEEREP_GRPC_ROS_JSON_POINT_DUMPER
#define SEEREP_GRPC_ROS_JSON_POINT_DUMPER

#include <curl/curl.h>
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

// logging
#include <boost/log/core.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>

namespace seerep_grpc_ros
{
class JsonPointDumper
{
public:
  JsonPointDumper(const std::string& filePath, const std::string& hdf5FilePath, const std::string& classesMappingPath);
  ~JsonPointDumper();

private:
  void readAndDumpJsonUos(const std::string& jsonFilePath);
  void readAndDumpJsonFr(const std::string& jsonFilePath, const std::string& classesMappingPath);
  flatbuffers::grpc::Message<seerep::fb::PointStamped>
  createPointForDetection(int32_t stampSecs, uint32_t stampNanos, const std::string& frameId,
                          const std::string& labelAgrovoc, const std::string& labelTrivial,
                          const std::string& instanceUUID, const double x, const double y, const double z,
                          const double diameter, const double confidence = 0.0);
  std::unordered_map<int64_t, std::string> readClassesMapping(const std::string& classesMappingPath);

  std::string translateNameToAgrovocConcept(std::string name);
  std::unordered_map<std::string, std::string> name2Concept;

  std::shared_ptr<seerep_hdf5_core::Hdf5CoreGeneral> ioCoreGeneral;
  std::shared_ptr<seerep_hdf5_fb::Hdf5FbPoint> ioPoint;

  std::string detectionCategory;
};

}  // namespace seerep_grpc_ros
#endif  // SEEREP_GRPC_ROS_JSON_POINT_DUMPER
