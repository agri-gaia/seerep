#ifndef SEEREP_CORE_CORE_IMAGE_H_
#define SEEREP_CORE_CORE_IMAGE_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/aabb.h>
#include <seerep-msgs/query.h>
#include <seerep-msgs/query-result.h>
// seerep-pb-io
#include <seerep-io-core/io-core-image.h>
// seerep-core
#include "core-tf.h"

// uuid
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.
#include <boost/lexical_cast.hpp>
#include <boost/functional/hash.hpp>

namespace seerep_core
{
class CoreImage
{
public:
  CoreImage(std::shared_ptr<seerep_io_core::IoCoreImage> hdf5_io, std::shared_ptr<seerep_core::CoreTf> tfOverview,
            std::string frameId);
  ~CoreImage();
  std::vector<boost::uuids::uuid> getData(const seerep_core_msgs::Query& bb);

  void addDataset(const seerep_core_msgs::DatasetIndexable& image);

private:
  void recreateDatasets();
  void addDatasetToIndices(const seerep_core_msgs::DatasetIndexable& img);

  void tryAddingDataWithMissingTF();
  std::vector<seerep_core_msgs::AabbIdPair> querySpatial(const seerep_core_msgs::Query& query);
  std::vector<seerep_core_msgs::AabbTimeIdPair> queryTemporal(const seerep_core_msgs::Query& query);
  std::set<boost::uuids::uuid> querySemantic(const seerep_core_msgs::Query& query);

  std::vector<boost::uuids::uuid> intersectQueryResults(std::vector<seerep_core_msgs::AabbIdPair> rt_result,
                                                        std::vector<seerep_core_msgs::AabbTimeIdPair> timetree_result,
                                                        std::set<boost::uuids::uuid> semanticResult);

  std::string m_frameId;
  std::shared_ptr<seerep_core::CoreTf> m_tfOverview;
  std::shared_ptr<seerep_io_core::IoCoreImage> m_hdf5_io;

  std::vector<std::shared_ptr<seerep_core_msgs::DatasetIndexable>> m_dataWithMissingTF;
  seerep_core_msgs::rtree m_rt;
  seerep_core_msgs::timetree m_timetree;
  std::unordered_map<std::string, std::vector<boost::uuids::uuid>> m_label;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_IMAGE_H_
