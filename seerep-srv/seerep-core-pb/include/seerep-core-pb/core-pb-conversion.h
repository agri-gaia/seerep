#ifndef SEEREP_CORE_PB_CONVERSION_H_
#define SEEREP_CORE_PB_CONVERSION_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/image.pb.h>
#include <seerep-msgs/query.pb.h>
#include <seerep-msgs/transform_stamped.pb.h>
#include <seerep-msgs/transform_stamped_query.pb.h>

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
  static seerep_core_msgs::Query fromPb(const seerep::Query& query, seerep_core_msgs::Datatype datatype);
  /**
   * @brief converts the protobuf image message to seerep core specific message
   * @param img the protobuf image message
   * @return the message in seerep core format for the data needed for the indices
   */
  static seerep_core_msgs::DatasetIndexable fromPb(const seerep::Image& img);

  /**
   * @brief converts the protobuf tf query message to seerep core specific message
   * @param query the protobuf tf query message
   * @return the tf query message in seerep core format
   */
  static seerep_core_msgs::QueryTf fromPb(const seerep::TransformStampedQuery& query);

private:
  /**
   * @brief converts the project part of the protobuf query message to seerep core specific message
   * @param query the protobuf query message
   * @param queryCore query message in seerep core format
   */
  static void fromPbProject(const seerep::Query& query, seerep_core_msgs::Query& queryCore);
  /**
   * @brief converts the label part of the protobuf query message to seerep core specific message
   * @param query the protobuf query message
   * @param queryCore query message in seerep core format
   */
  static void fromPbLabel(const seerep::Query& query, seerep_core_msgs::Query& queryCore);
  /**
   * @brief converts the temporal part of the protobuf query message to seerep core specific message
   * @param query the protobuf query message
   * @param queryCore query message in seerep core format
   */
  static void fromPbTime(const seerep::Query& query, seerep_core_msgs::Query& queryCore);
  /**
   * @brief converts the spatial part of the protobuf query message to seerep core specific message
   * @param query the protobuf query message
   * @param queryCore query message in seerep core format
   */
  static void fromPbBoundingBox(const seerep::Query& query, seerep_core_msgs::Query& queryCore);
};

}  // namespace seerep_core_pb

#endif  // SEEREP_CORE_PB_CONVERSION_H_
