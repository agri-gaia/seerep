#ifndef SEEREP_CORE_FB_CONVERSION_H_
#define SEEREP_CORE_FB_CONVERSION_H_

#include <functional>
#include <optional>

// grpc / flatbuffer
#include <flatbuffers/grpc.h>
// seerep-msgs
#include <seerep-msgs/image_generated.h>
#include <seerep-msgs/point_stamped_generated.h>
#include <seerep-msgs/query_generated.h>
#include <seerep-msgs/transform_stamped_generated.h>
#include <seerep-msgs/transform_stamped_query_generated.h>
#include <seerep-msgs/uuids_per_project_generated.h>
// seerep-core-msgs
#include <seerep-msgs/dataset-indexable.h>
#include <seerep-msgs/query-result.h>
#include <seerep-msgs/query-tf.h>
#include <seerep-msgs/query.h>
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
   * @brief converts the flatbuffer query message to seerep core specific message
   * @param query the flatbuffer query message
   * @return the query message in seerep core format
   */
  static seerep_core_msgs::Query fromFb(const seerep::fb::Query& query);
  /**
   * @brief converts the flatbuffer image message to seerep core specific message
   * @param img the flatbuffer image message
   * @return the message in seerep core format for the data needed for the indices
   */
  static seerep_core_msgs::DatasetIndexable fromFb(const seerep::fb::Image& img);
  /**
   * @brief converts the flatbuffer point message to seerep core specific message
   * @param img the flatbuffer point message
   * @return the message in seerep core format for the data needed for the indices
   */
  static seerep_core_msgs::DatasetIndexable fromFb(const seerep::fb::PointStamped& point);

  /**
   * @brief converts the flatbuffer tf query message to seerep core specific message
   * @param query the flatbuffer tf query message
   * @return the tf query message in seerep core format
   */
  static seerep_core_msgs::QueryTf fromFb(const seerep::fb::TransformStampedQuery& query);

  /**
   * @brief converts the query result from the seerep core format to gRPC flatbuffer message
   * @param result the query result in seerep core specific format
   * @return the result as gRPC flatbuffer message
   */
  static flatbuffers::grpc::Message<seerep::fb::UuidsPerProject> toFb(seerep_core_msgs::QueryResult& result);

private:
  /**
   * @brief converts the project part of the flatbuffer query message to seerep core specific message
   * @param query the flatbuffer query message
   * @param queryCore query message in seerep core format
   */
  static void fromFbQueryProject(const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>* projects,
                                 std::optional<std::vector<boost::uuids::uuid>>& queryCoreProjects);
  /**
   * @brief converts the label part of the flatbuffer query message to seerep core specific message
   * @param query the flatbuffer query message
   * @param queryCore query message in seerep core format
   */
  static void fromFbQueryLabel(const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>* label,
                               std::optional<std::vector<std::string>>& queryCoreLabel);
  /**
   * @brief converts the temporal part of the flatbuffer query message to seerep core specific message
   * @param query the flatbuffer query message
   * @param queryCore query message in seerep core format
   */
  static void fromFbQueryTime(const seerep::fb::TimeInterval* time,
                              std::optional<seerep_core_msgs::Timeinterval>& queryCoreTime);
  /**
   * @brief converts the spatial part of the flatbuffer query message to seerep core specific message
   * @param query the flatbuffer query message
   * @param queryCore query message in seerep core format
   */
  static void fromFbQueryBoundingBox(const seerep::fb::Boundingbox* boundingBox,
                                     std::optional<seerep_core_msgs::AABB>& queryCoreBoundingBox,
                                     std::string& queryCoreHeaderFrameId);

  static void fromFbDataHeader(seerep_core_msgs::Header& coreHeader, const seerep::fb::Header* header);

  static boost::uuids::uuid fromFbDataHeaderUuid(const std::string& uuidMsg);

  static void
  fromFbDataLabelsGeneral(std::vector<seerep_core_msgs::LabelWithInstance>& labelWithInstance,
                          const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::LabelWithInstance>>* labelsGeneral);
  static void
  fromFbDataLabelsGeneral(std::vector<seerep_core_msgs::LabelWithInstance>& labelWithInstance,
                          const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeled>>* labelsBB2d);
};

}  // namespace seerep_core_fb

#endif  // SEEREP_CORE_FB_CONVERSION_H_
