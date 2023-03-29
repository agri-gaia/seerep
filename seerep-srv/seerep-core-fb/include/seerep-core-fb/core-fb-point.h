#ifndef SEEREP_CORE_FB_POINT_H_
#define SEEREP_CORE_FB_POINT_H_

#include <functional>
#include <optional>

#include "core-fb-conversion.h"
#include "core-fb-general.h"

// seerep_msgs
#include <seerep_msgs/attributes_stamped_generated.h>
#include <seerep_msgs/point_stamped_generated.h>
#include <seerep_msgs/query_generated.h>

// seerep_core_msgs
#include <seerep_msgs/query.h>
#include <seerep_msgs/query_result.h>

// seerep_hdf5_fb
#include <seerep_hdf5_fb/hdf5_fb_point.h>

// seerep-core
#include <seerep_core/core.h>

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_core_fb
{
/**
 * @brief This class is the center piece between the gRPC interface, the core and the hdf5-io for points
 *
 * The functions of this class are called by the corresponding gRPC services. The data storing and loading is done
 * directly via the hdf5-io-fb. When adding new data the needed information for the indices are disclosed to the core.
 * When data is queried the core is consulted to get the UUIDs of the data answering the query. The data is then loaded
 * from this class via the hdf5-io.
 */
class CoreFbPoint
{
public:
  /**
   * @brief Constructs the point specific object based on the general core
   * @param seerepCore a shared pointer to the general core
   */
  CoreFbPoint(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CoreFbPoint();

  /**
   * @brief Function to query points
   * @param query the flatbuffer query
   * @param writer the writer object used to send the points matching the query directly via gRPC
   *
   * Based on the query the indices are used to get the uuids of the points matching the query. Then the points are
   * loaded by the hdf5-fb-io and send via gRPC directly using the writer
   */
  void getData(const seerep::fb::Query* query,
               grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::PointStamped>>* const writer);
  /**
   * @brief Add an point to the indices and write the data to hdf5
   * @param img the flatbuffer message containing the point
   * @return the uuid of the stored point
   *
   * The point is stored in the hdf5 file via hdf5-io-fb. The data needed for the indices is extracted and added to the
   * core. If the uuid of point is not defined yet, a uuid is generated and returned.
   */
  boost::uuids::uuid addData(const seerep::fb::PointStamped* point);
  /**
   * @brief Adds attributes to an existing point
   * @param uuidAttributePair a pair of the uuid of the targeted point and the attributes
   */
  void addAttributes(const seerep::fb::AttributesStamped& attributesStamped);

private:
  /** @brief a shared pointer to the general core */
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  /** a map from the uuids of the projects to the hdf5-io objects handling the io for the object */
  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_hdf5_fb::Hdf5FbPoint>, boost::hash<boost::uuids::uuid>>
      m_hdf5IoMap;
  /** the logger for the logging framework */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

}  // namespace seerep_core_fb

#endif  // SEEREP_CORE_FB_POINT_H_
