#ifndef SEEREP_CORE_FB_CONVERSION_H_
#define SEEREP_CORE_FB_CONVERSION_H_

#include <functional>
#include <optional>

// grpc / flatbuffer
#include <flatbuffers/flatbuffers.h>
#include <flatbuffers/grpc.h>
#include <seerep_msgs/camera_intrinsics_generated.h>
#include <seerep_msgs/camera_intrinsics_query_generated.h>
// seerep-msgs
#include <seerep_msgs/aabb.h>
#include <seerep_msgs/camera_intrinsics.h>
#include <seerep_msgs/camera_intrinsics_query.h>
#include <seerep_msgs/datatype_generated.h>
#include <seerep_msgs/image_generated.h>
#include <seerep_msgs/point_cloud_2_generated.h>
#include <seerep_msgs/point_stamped_generated.h>
#include <seerep_msgs/project_info.h>
#include <seerep_msgs/project_info_generated.h>
#include <seerep_msgs/project_infos_generated.h>
#include <seerep_msgs/query_generated.h>
#include <seerep_msgs/query_instance_generated.h>
#include <seerep_msgs/region_of_interest.h>
#include <seerep_msgs/transform_stamped_generated.h>
#include <seerep_msgs/transform_stamped_query_generated.h>
#include <seerep_msgs/uuids_per_project_generated.h>

// seerep_core_msgs
#include <seerep_msgs/dataset_indexable.h>
#include <seerep_msgs/query.h>
#include <seerep_msgs/query_result.h>
#include <seerep_msgs/query_tf.h>

// ros
#include <geometry_msgs/TransformStamped.h>

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

namespace seerep_core_fb
{
/**
 * @brief This class converts between flatbuffer messages and seerep core specific messages
 *
 */
class CoreFbConversion
{
public:
  /**
   * @brief converts the flatbuffer instance query message to seerep core specific message
   * @param query the flatbuffer instance query message
   * @return the query message in seerep core format
   */
  static seerep_core_msgs::Query fromFb(const seerep::fb::QueryInstance* queryInstance);
  /**
   * @brief converts the flatbuffer query message to seerep core specific message
   * @param datatype the flatbuffer query message
   * @return the query message in seerep core format
   */
  static seerep_core_msgs::Datatype fromFb(const seerep::fb::Datatype& datatype);
  /**
   * @brief converts the flatbuffer query message to seerep core specific message
   * @param query the flatbuffer query message
   * @param datatype the targeted datatype
   * @return the query message in seerep core format
   */
  static seerep_core_msgs::Query fromFb(const seerep::fb::Query* query, const seerep_core_msgs::Datatype& datatype);
  /**
   * @brief converts the flatbuffer image message to seerep core specific message
   * @param img the flatbuffer image message
   * @return the message in seerep core format for the data needed for the indices
   */
  static seerep_core_msgs::DatasetIndexable fromFb(const seerep::fb::Image& img);
  /**
   * @brief converts the flatbuffer point message to seerep core specific message
   * @param point the flatbuffer point message
   * @return the message in seerep core format for the data needed for the indices
   */
  static seerep_core_msgs::DatasetIndexable fromFb(const seerep::fb::PointStamped* point);

  /**
   * @brief converts the flatbuffer point cloud message to seerep core specific message
   * @param cloud the flatbuffer point cloud message
   * @return the message in the seerep core format for the data needed for the indices
   */
  static seerep_core_msgs::DatasetIndexable fromFb(const seerep::fb::PointCloud2& cloud);

  /**
   * @brief converts the flatbuffer tf query message to seerep core specific message
   * @param query the flatbuffer tf query message
   * @return the tf query message in seerep core format
   */
  static seerep_core_msgs::QueryTf fromFb(const seerep::fb::TransformStampedQuery& query);

  /**
   * @brief converts the flatbuffer camera intrinsics message to seerep core specific message
   * @param ci flatbuffer camera intrinsics message
   * @return camera intrinsics message in seerep core format
   */
  static seerep_core_msgs::camera_intrinsics fromFb(const seerep::fb::CameraIntrinsics& ci);

  /**
   * @brief Convert seerep core message header to flatbuffer header.
   *
   * @param mb Message builder to be used for building the message
   * @param header seerep core header to convert
   * @return flatbuffers::Offset<seerep::fb::Header>
   */
  static flatbuffers::Offset<seerep::fb::Header> toFb(flatbuffers::grpc::MessageBuilder& mb,
                                                      const seerep_core_msgs::Header header);

  /**
   * @brief Convert seerep core message region of interest to flatbuffer region of interest.
   *
   * @param mb Message builder to be used for building the message
   * @param roi seerep core region of interest to convert
   * @return flatbuffers::Offset<seerep::fb::regionOfInterest>
   */
  static flatbuffers::Offset<seerep::fb::RegionOfInterest> toFb(flatbuffers::grpc::MessageBuilder& mb,
                                                                const seerep_core_msgs::region_of_interest& roi);

  /**
   * @brief Convert seerep core message timestamp to flatbuffer timestamp.
   *
   * @param mb Message builder to be used for building the message
   * @param ts seerep core timestamp to convert
   * @return flatbuffers::Offset<seerep::fb::Timestamp>
   */
  static flatbuffers::Offset<seerep::fb::Timestamp> toFb(flatbuffers::grpc::MessageBuilder& mb,
                                                         const seerep_core_msgs::Timestamp ts);

  /**
   * @brief Convert seerep core message camera intrinsics to flatbuffer camera intrinsics.
   *
   * @param mb Message builder to be used for building the message
   * @param ci seerep core camera intrinsics to convert
   * @return flatbuffers::Offset<seerep::fb::CameraIntrinsics>
   */
  static flatbuffers::Offset<seerep::fb::CameraIntrinsics> toFb(flatbuffers::grpc::MessageBuilder& mb,
                                                                const seerep_core_msgs::camera_intrinsics ci);

  /**
   * @brief Converts a seerep core project info message into the corresponding flatbuffer message
   *
   * @param mb Flatbuffers message builder
   * @param prjInfo seerep core project info message to convert
   * @return flatbuffers::Offset<seerep::fb::ProjectInfo>
   */
  static flatbuffers::Offset<seerep::fb::ProjectInfo> toFb(flatbuffers::grpc::MessageBuilder& fbb,
                                                           const seerep_core_msgs::ProjectInfo& prjInfo);

  static flatbuffers::Offset<seerep::fb::ProjectInfos> toFb(flatbuffers::grpc::MessageBuilder& fbb,
                                                            const std::vector<seerep_core_msgs::ProjectInfo>& prjInfos);

  /**
   * @brief converts the flatbuffer region of interest message to the specific seerep core message
   * @param query the flatbuffer region of interest message
   * @return the region of interest message in seerep core format
   */
  static seerep_core_msgs::region_of_interest fromFb(const seerep::fb::RegionOfInterest& roi);

  /**
   * @brief converts the flatbuffer camera intrinsics message to the specific seerep core message
   *
   * @param camIntrinsicsQuery
   * @return seerep_core_msgs::camera_intrinsics_query
   */
  static seerep_core_msgs::camera_intrinsics_query fromFb(const seerep::fb::CameraIntrinsicsQuery& camIntrinsicsQuery);

  static flatbuffers::Offset<seerep::fb::Boundingbox> toFb(flatbuffers::grpc::MessageBuilder& mb,
                                                           seerep_core_msgs::AABB& aabb);

  static flatbuffers::Offset<seerep::fb::TimeInterval> toFb(flatbuffers::grpc::MessageBuilder& mb,
                                                            seerep_core_msgs::AabbTime& aabb);

  /**
   * @brief converts the query result from the seerep core format to gRPC flatbuffer message
   * @param result the query result in seerep core specific format
   * @return the result as gRPC flatbuffer message
   */
  static flatbuffers::grpc::Message<seerep::fb::UuidsPerProject> toFb(seerep_core_msgs::QueryResult& result);

  static std::vector<seerep_core_msgs::Datatype> fromFbDatatypeVector(const seerep::fb::Datatype& dt);

private:
  /**
   * @brief converts the project part of the flatbuffer query message to seerep core specific message
   * @param query the flatbuffer query message
   * @param queryCoreProjects the projects in the query message in seerep core format
   */
  static void fromFbQueryProject(const seerep::fb::Query* query,
                                 std::optional<std::vector<boost::uuids::uuid>>& queryCoreProjects);
  /**
   * @brief converts the instance part of the flatbuffer query message to seerep core specific message
   * @param query the flatbuffer query message
   * @param queryCoreInstances the instances in the query message in seerep core format
   */
  static void fromFbQueryInstance(const seerep::fb::Query* query,
                                  std::optional<std::vector<boost::uuids::uuid>>& queryCoreInstances);
  /**
   * @brief converts the instance part of the flatbuffer query message to seerep core specific message
   * @param query the flatbuffer query message
   * @param queryCoreDataUuids the data uuids in the query message in seerep core format
   */
  static void fromFbQueryDataUuids(const seerep::fb::Query* query,
                                   std::optional<std::vector<boost::uuids::uuid>>& queryCoreDataUuids);
  /**
   * @brief converts the label part of the flatbuffer query message to seerep core specific message
   * @param query the flatbuffer query message
   * @param queryCoreLabel the labels (per category) in the query message in seerep core format
   */
  static void fromFbQueryLabel(const seerep::fb::Query* query,
                               std::optional<std::unordered_map<std::string, std::vector<std::string>>>& queryCoreLabel);
  /**
   * @brief converts the temporal part of the flatbuffer query message to seerep core specific message
   * @param query the flatbuffer query message
   * @param queryCoreTime the time in the query message in seerep core format
   */
  static void fromFbQueryTime(const seerep::fb::Query* query,
                              std::optional<seerep_core_msgs::Timeinterval>& queryCoreTime);
  /**
   * @brief converts the spatial part of the flatbuffer query message to seerep core specific message
   * @param query the flatbuffer query message
   * @param queryCoreBoundingBox the bounding box in the query message in seerep core format
   * @param queryCoreHeaderFrameId the frame id in the header of the query message in the seerep core format
   */
  static void fromFbQueryBoundingBox(const seerep::fb::Query* query,
                                     std::optional<seerep_core_msgs::AABB>& queryCoreBoundingBox,
                                     std::string& queryCoreHeaderFrameId);
  /**
   * @brief extracts the WithoutData Flag of the flatbuffer query message
   * @param query the flatbuffer query message
   * @return flag if the data should NOT be loaded
   */
  static bool fromFbQueryWithoutData(const seerep::fb::Query* query);

  /**
   * @brief extracts the mustHaveAllLabels Flag of the flatbuffer query message
   * @param query the flatbuffer query message
   * @return flag if all labels must be present to fit semantic query
   */
  static bool fromFbQueryMustHaveAllLabels(const seerep::fb::Query* query);

  /**
   * @brief extracts the maxNumData of the flatbuffer query message
   *
   * @param query the flatbuffer query message
   * @return uint max number of datasets that should be returned
   */
  static uint fromFbQueryMaxNumData(const seerep::fb::Query* query);

  /**
   * @brief converts the header of the flatbuffer data message to seerep core specific message
   * @param header the header in the flatbuffer data message
   * @param coreHeader the header in the data message in seerep core format
   * @param datatype of the associated message
   */
  static void fromFbDataHeader(const seerep::fb::Header* header, seerep_core_msgs::Header& coreHeader,
                               seerep_core_msgs::Datatype&& datatype);

  /**
   * @brief converts (or generates if not set) the msg uuid of the flatbuffer data message to seerep core specific message
   * @param uuidMsg the msg uuid in the flatbuffer data message
   */
  static boost::uuids::uuid fromFbDataHeaderUuid(const std::string& uuidMsg);
  /**
   * @brief converts the labels_general with instances of the flatbuffer data message to seerep core specific message
   * @param labelsGeneral the labels_general with instances in the flatbuffer data message
   * @param labelsWithInstancesWithCategory the labels_general with instances (per category) in the data message in
   * seerep core format
   */
  static void fromFbDataLabelsGeneral(
      const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::LabelsWithInstanceWithCategory>>* labelsGeneral,
      std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>>& labelsWithInstancesWithCategory);
  /**
   * @brief converts the BoundingBox2DLabeled with instances of the flatbuffer data message to seerep core specific message
   * @param labelsGeneral the BoundingBox2DLabeled with instances in the flatbuffer data message
   * @param labelWithInstance the BoundingBox2DLabeled with instances in the data message in seerep core format
   */
  static void fromFbDataLabelsBb2d(
      const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeledWithCategory>>* labelsBB2d,
      std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>>& labelsWithInstancesWithCategory);
  /**
   * @brief converts the BoundingBoxLabeled with instances of the flatbuffer data message to seerep core specific message
   * @param labelsBB the BoundingBoxLabeled with instances in the flatbuffer data message
   * @param labelWithInstance the BoundingBoxLabeled with instances in the data message in seerep core format
   */
  static void fromFbDataLabelsBb(
      const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeledWithCategory>>* labelsBB,
      std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>>& labelsWithInstancesWithCategory);
};

}  // namespace seerep_core_fb

#endif  // SEEREP_CORE_FB_CONVERSION_H_
