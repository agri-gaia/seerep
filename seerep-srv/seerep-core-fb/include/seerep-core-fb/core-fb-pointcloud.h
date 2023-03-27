#ifndef SEEREP_CORE_FB_POINTCLOUD_H_
#define SEEREP_CORE_FB_POINTCLOUD_H_

#include "core-fb-conversion.h"
#include "core-fb-general.h"

// seerep_msgs
#include <seerep_msgs/point_cloud_2_generated.h>
#include <seerep_msgs/query_generated.h>

// seerep_core_msgs
#include <seerep_msgs/query.h>
#include <seerep_msgs/query_result.h>

// seerep-hdf5-pb
#include <seerep-hdf5-fb/hdf5-fb-pointcloud.h>
// seerep-core
#include <seerep-core/core.h>

// uuid

#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

namespace seerep_core_fb
{
/**
 * @brief This class is the center piece between the gRPC interface, the core and the hdf5-io for point clouds
 *
 * The functions of this class are called by the corresponding gRPC services. The data storing and loading is done
 * directly via the hdf5-io-fb. When adding new data the needed information for the indices are disclosed to the core.
 * When data is queried the core is consulted to get the UUIDs of the data answering the query. The data is then
 * loaded from this class via the hdf5-io.
 */
class CoreFbPointCloud
{
public:
  /**
   * @brief Constructs the point cloud specific object based on the general core
   *
   * @param seerepCore a shared pointer to the general core
   */
  CoreFbPointCloud(std::shared_ptr<seerep_core::Core> seerepCore);

  /**
   * @brief Function to query point clouds
   *
   * @param query the flatbuffer query
   * @param writer the writer object used to send the point clouds matching the query directly via gRPC
   *
   * Based on the query the indices are used to get the uuids of the point clouds matching the query. Then the point
   * clouds are loaded by the hdf5-fb-io and send via gRPC directly using the writer
   */
  void getData(const seerep::fb::Query* query,
               grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::PointCloud2>>* const writer);
  /**
   * @brief Add a point cloud to the indices and write the data to hdf5
   *
   * @param pc the point cloud message to index and save
   * @return boost::uuids::uuid the uuid of the stored image
   */
  boost::uuids::uuid addData(const seerep::fb::PointCloud2& pc);

private:
  /** @brief a shared pointer to the general core */
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  /** @brief a map from the uuids of the project to the hdf5-io objects handling the io for the object*/
  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_hdf5_fb::Hdf5FbPointCloud>,
                     boost::hash<boost::uuids::uuid>>
      m_hdf5IoMap;
  /** @brief the logger for the logging framework*/
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

}  // namespace seerep_core_fb

#endif  // SEEREP_CORE_FB_POINTCLOUD_H_
