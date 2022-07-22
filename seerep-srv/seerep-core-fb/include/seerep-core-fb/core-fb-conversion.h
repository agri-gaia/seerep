#ifndef SEEREP_CORE_FB_CONVERSION_H_
#define SEEREP_CORE_FB_CONVERSION_H_

#include <functional>
#include <optional>

// grpc / flatbuffer
#include <flatbuffers/grpc.h>
// seerep-msgs
#include <seerep-msgs/image_generated.h>
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
  static void fromFbProject(const seerep::fb::Query& query, seerep_core_msgs::Query& queryCore);
  /**
   * @brief converts the label part of the flatbuffer query message to seerep core specific message
   * @param query the flatbuffer query message
   * @param queryCore query message in seerep core format
   */
  static void fromFbLabel(const seerep::fb::Query& query, seerep_core_msgs::Query& queryCore);
  /**
   * @brief converts the temporal part of the flatbuffer query message to seerep core specific message
   * @param query the flatbuffer query message
   * @param queryCore query message in seerep core format
   */
  static void fromFbTime(const seerep::fb::Query& query, seerep_core_msgs::Query& queryCore);
  /**
   * @brief converts the spatial part of the flatbuffer query message to seerep core specific message
   * @param query the flatbuffer query message
   * @param queryCore query message in seerep core format
   */
  static void fromFbBoundingBox(const seerep::fb::Query& query, seerep_core_msgs::Query& queryCore);
};

}  // namespace seerep_core_fb

#endif  // SEEREP_CORE_FB_CONVERSION_H_
