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
class CoreFbConversion
{
public:
  static seerep_core_msgs::Query fromFb(const seerep::fb::Query& query);
  static seerep_core_msgs::DatasetIndexable fromFb(const seerep::fb::Image& img);

  static seerep_core_msgs::QueryTf fromFb(const seerep::fb::TransformStampedQuery& query);

  static flatbuffers::grpc::Message<seerep::fb::UuidsPerProject> toFb(seerep_core_msgs::QueryResult& result);
};

}  // namespace seerep_core_fb

#endif  // SEEREP_CORE_FB_CONVERSION_H_
