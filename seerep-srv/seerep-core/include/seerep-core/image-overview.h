#ifndef SEEREP_CORE_IMAGE_OVERVIEW_H_
#define SEEREP_CORE_IMAGE_OVERVIEW_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/aabb.h>
#include <seerep-msgs/query.h>
#include <seerep-msgs/query-result.h>
// seerep-pb-io
#include <seerep-core-io/image-io-core.h>
// seerep-conversion
#include <seerep_ros_conversions/conversions.h>

// seerep-core
#include "tf-overview.h"

// uuid
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <boost/lexical_cast.hpp>
#include <boost/functional/hash.hpp>

namespace seerep_core
{
class ImageOverview
{
public:
  ImageOverview(std::shared_ptr<seerep_core_io::ImageIOCore> hdf5_io,
                std::shared_ptr<seerep_core::TFOverview> tfOverview, std::string frameId);
  ~ImageOverview();
  std::vector<boost::uuids::uuid> getData(const seerep_core_msgs::Query& bb);

  boost::uuids::uuid addDataset(seerep_core_msgs::DatasetIndexable& image);

private:
  void recreateDatasets();
  void addImageToIndices(seerep_core_msgs::DatasetIndexable& img);

  void tryAddingDataWithMissingTF();
  std::vector<seerep_core_msgs::AabbIdPair> querySpatial(const seerep_core_msgs::Query& query);
  std::vector<seerep_core_msgs::AabbTimeIdPair> queryTemporal(const seerep_core_msgs::Query& query);
  std::set<boost::uuids::uuid> querySemantic(const seerep_core_msgs::Query& query);

  std::vector<boost::uuids::uuid> intersectQueryResults(std::vector<seerep_core_msgs::AabbIdPair> rt_result,
                                                        std::vector<seerep_core_msgs::AabbTimeIdPair> timetree_result,
                                                        std::set<boost::uuids::uuid> semanticResult);

  std::string m_frameId;
  std::shared_ptr<seerep_core::TFOverview> m_tfOverview;
  std::shared_ptr<seerep_core_io::ImageIOCore> m_hdf5_io;

  std::vector<std::shared_ptr<seerep_core_msgs::DatasetIndexable>> m_dataWithMissingTF;
  seerep_core_msgs::rtree m_rt;
  seerep_core_msgs::timetree m_timetree;
  std::unordered_map<std::string, std::vector<boost::uuids::uuid>> m_label;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_IMAGE_OVERVIEW_H_
