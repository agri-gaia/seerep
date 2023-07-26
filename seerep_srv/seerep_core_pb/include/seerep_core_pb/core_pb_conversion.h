#ifndef SEEREP_CORE_PB_CONVERSION_H_
#define SEEREP_CORE_PB_CONVERSION_H_

#include <functional>
#include <optional>

// seerep_msgs
#include <seerep_msgs/camera_intrinsics.pb.h>
#include <seerep_msgs/camera_intrinsics_query.pb.h>
#include <seerep_msgs/datatype.pb.h>
#include <seerep_msgs/image.pb.h>
#include <seerep_msgs/point_cloud_2.pb.h>
#include <seerep_msgs/query.pb.h>
#include <seerep_msgs/region_of_interest.pb.h>
#include <seerep_msgs/transform_stamped.pb.h>
#include <seerep_msgs/transform_stamped_query.pb.h>

// seerep_core_msgs
#include <seerep_msgs/camera_intrinsics.h>
#include <seerep_msgs/camera_intrinsics_query.h>
#include <seerep_msgs/dataset_indexable.h>
#include <seerep_msgs/query.h>
#include <seerep_msgs/query_result.h>
#include <seerep_msgs/query_tf.h>
#include <seerep_msgs/region_of_interest.h>

// ros
#include <geometry_msgs/TransformStamped.h>

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

namespace seerep_core_pb
{
/**
 * @brief This class converts between protobuf messages and seerep core specific messages
 *
 */
class CorePbConversion
{
public:
  /**
   * @brief converts the protobuf query message to seerep core specific message
   * @param query the protobuf query message
   * @return the query message in seerep core format
   */
  static seerep_core_msgs::Query fromPb(const seerep::pb::Query& query, seerep_core_msgs::Datatype datatype);
  /**
   * @brief converts the protobuf image message to seerep core specific message
   * @param img the protobuf image message
   * @return the message in seerep core format for the data needed for the indices
   */
  static seerep_core_msgs::DatasetIndexable fromPb(const seerep::pb::Image& img);

  /**
   * @brief converts the protobuf tf query message to seerep core specific message
   * @param query the protobuf tf query message
   * @return the tf query message in seerep core format
   */
  static seerep_core_msgs::QueryTf fromPb(const seerep::pb::TransformStampedQuery& query);

  static seerep_core_msgs::region_of_interest fromPb(const seerep::pb::RegionOfInterest& roi);
  static seerep::pb::RegionOfInterest toPb(const seerep_core_msgs::region_of_interest& roi);

  static seerep_core_msgs::Header fromPb(const seerep::pb::Header& header);
  static seerep::pb::Header toPb(const seerep_core_msgs::Header& header);

  static seerep_core_msgs::Timestamp fromPb(const seerep::pb::Timestamp& timestamp);
  static seerep::pb::Timestamp toPb(const seerep_core_msgs::Timestamp& timestamp);

  static seerep_core_msgs::camera_intrinsics fromPb(const seerep::pb::CameraIntrinsics& camintrinsics);
  static seerep::pb::CameraIntrinsics toPb(const seerep_core_msgs::camera_intrinsics& camintrinsics);

  static seerep_core_msgs::camera_intrinsics_query fromPb(const seerep::pb::CameraIntrinsicsQuery& camintrinsics_query);
  /**
   * @brief converts a seerep core msg AabbbTime to protobuf time inverval
   *
   * @param timeinterval core msg AabbTime
   * @param ti_bb pointer to protobuf time interval
   */
  static void toPb(const seerep_core_msgs::AabbTime& timeinterval, seerep::pb::TimeInterval* ti_pb);

  /**
   * @brief converts a seerep core msg aabb to protobuf aabb
   *
   * @param aabb core msg aabb
   * @param bb_bb pointer to protobuf aabb
   */
  static void toPb(const seerep_core_msgs::AABB& aabb, seerep::pb::Boundingbox* bb_pb);

  /**
   * @brief Convert Pb datatype to a vector containing core message datatypes
   *
   * @param datatype protobuf datatype
   * @param dtCore std::vector<seerep_core_msgs::Datatype> vector of core msg datatype
   */
  static void fromPbDatatypeVector(const seerep::datatype& datatype, std::vector<seerep_core_msgs::Datatype>& dtCore);

private:
  /**
   * @brief converts the project part of the protobuf query message to seerep core specific message
   * @param query the protobuf query message
   * @param queryCore query message in seerep core format
   */
  static void fromPbProject(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore);
  /**
   * @brief converts the label part of the protobuf query message to seerep core specific message
   * @param query the protobuf query message
   * @param queryCore query message in seerep core format
   */
  static void fromPbLabel(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore);
  /**
   * @brief converts the temporal part of the protobuf query message to seerep core specific message
   * @param query the protobuf query message
   * @param queryCore query message in seerep core format
   */
  static void fromPbTime(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore);
  /**
   * @brief converts the spatial part of the protobuf query message to seerep core specific message
   * @param query the protobuf query message
   * @param queryCore query message in seerep core format
   */
  static void fromPbBoundingBox(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore);
  /**
   * @brief converts seerep probotbuf point2d to core msg point2d
   *
   * @param point protobuf point2d msg
   * @return seerep_core_msgs::Point2D core point2d msg
   */
  static seerep_core_msgs::Point2D fromPbPoint2D(const seerep::pb::Point2D& point);
  /**
   * @brief
   *
   * @param query
   * @param queryCore
   */
  static void fromPbPolygon(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore);
  /**
   * @brief converts the mustHaveAllLabels flag of the protobuf query message to seerep core specific message
   * @param query the protobuf query message
   * @param queryCore query message in seerep core format
   */
  static void fromPbMustHaveAllLabels(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore);
  /**
   * @brief converts the inMapFrame flag of the protobuf query message to seerep core specific message
   * @param query the protobuf query message
   * @param queryCore query message in seerep core format
   */
  static void fromPbInMapFrame(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore);
  /**
   * @brief converts the fullyEncapsulated flag of the protobuf query message to seerep core specific message
   * @param query the protobuf query message
   * @param queryCore query message in seerep core format
   */
  static void fromPbFullyEncapsulated(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore);
  /**
   * @brief converts the instance uuids of the protobuf query message to seerep core specific message
   * @param query the protobuf query message
   * @param queryCore query message in seerep core format
   */
  static void fromPbInstance(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore);
  /**
   * @brief converts the data uuids of the protobuf query message to seerep core specific message
   * @param query the protobuf query message
   * @param queryCore query message in seerep core format
   */
  static void fromPbDataUuids(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore);
  /**
   * @brief converts the withoutData flag of the protobuf query message to seerep core specific message
   * @param query the protobuf query message
   * @param queryCore query message in seerep core format
   */
  static void fromPbWithOutData(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore);

  /**
   * @brief extracts the maxNumData of the flatbuffer query message
   *
   * @param query the protobuf query message
   * @param queryCore query message in seerep core format
   */
  static void fromFbQueryMaxNumData(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore);
};

}  // namespace seerep_core_pb

#endif  // SEEREP_CORE_PB_CONVERSION_H_
