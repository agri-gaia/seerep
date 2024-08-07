#ifndef SEEREP_CORE_FB_IMAGE_H_
#define SEEREP_CORE_FB_IMAGE_H_

#include <functional>
#include <optional>

// seerep_core_fb
#include "core_fb_conversion.h"
#include "core_fb_general.h"

// seerep_msgs
#include <seerep_msgs/dataset_uuid_label_generated.h>
#include <seerep_msgs/image_generated.h>
#include <seerep_msgs/query_generated.h>

// seerep_core_msgs
#include <seerep_msgs/query.h>
#include <seerep_msgs/query_result.h>

// seerep_hdf5_fb
#include <seerep_hdf5_fb/hdf5_fb_image.h>

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
 * @brief This class is the center piece between the gRPC interface, the core
 * and the hdf5-io for images
 *
 * The functions of this class are called by the corresponding gRPC services.
 * The data storing and loading is done directly via the hdf5-io-fb. When adding
 * new data the needed information for the indices are disclosed to the core.
 * When data is queried the core is consulted to get the UUIDs of the data
 * answering the query. The data is then loaded from this class via the hdf5-io.
 */
class CoreFbImage
{
public:
  /**
   * @brief Constructs the image specific object based on the general core
   * @param seerepCore a shared pointer to the general core
   */
  CoreFbImage(std::shared_ptr<seerep_core::Core> seerepCore);
  ~CoreFbImage();

  /**
   * @brief Function to query images
   * @param query the flatbuffer query
   * @param writer the writer object used to send the images matching the query
   * directly via gRPC
   *
   * Based on the query the indices are used to get the uuids of the images
   * matching the query. Then the images are loaded by the hdf5-fb-io and send
   * via gRPC directly using the writer
   */
  void getData(
      const seerep::fb::Query* query,
      grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::Image>>* const
          writer);
  /**
   * @brief Write image data to hdf5
   * @param img the flatbuffer message containing the image
   * @return the uuid of the stored image
   *
   * The image is stored in the hdf5 file via hdf5-io-fb.
   * If the uuid of image is not defined yet, a uuid is generated and returned.
   */
  boost::uuids::uuid addDataToHdf5(const seerep::fb::Image& img);

  /**
   * @brief Extract image data from hdf5 and build the indices.
   *
   * This method complements @ref addDataToHdf5 and should be called after
   * the data has been added to hdf5. The data for the indices is retrieved from
   * hdf5 and added to the indices to the core.
   *
   * @param projectImgUuids a vector containing image uuids mapped to the
   * projects where they reside. The first slot contains the project uuid and
   * the second slot the image uuid.
   *
   * @see seerep_core_fb::CoreFbImage::addDataToHdf5
   */
  void
  buildIndices(const std::vector<std::pair<std::string, boost::uuids::uuid>>&
                   projectImgUuids);

  /**
   * @brief Adds labels to an existing image
   * @param datasetUuidLabel the flatbuffer message containing  labels
   */
  void addLabel(const seerep::fb::DatasetUuidLabel& datasetUuidLabel);

private:
  /** @brief a shared pointer to the general core */
  std::shared_ptr<seerep_core::Core> m_seerepCore;
  /** a map from the uuids of the projects to the hdf5-io objects handling the io for the object */
  std::unordered_map<boost::uuids::uuid,
                     std::shared_ptr<seerep_hdf5_fb::Hdf5FbImage>,
                     boost::hash<boost::uuids::uuid>>
      m_hdf5IoMap;
  /** the logger for the logging framework */
  boost::log::sources::severity_logger<boost::log::trivial::severity_level>
      m_logger;
};

}  // namespace seerep_core_fb

#endif  // SEEREP_CORE_FB_IMAGE_H_
