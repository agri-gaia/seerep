#ifndef SEEREP_CORE_CORE_INSTANCE_H_
#define SEEREP_CORE_CORE_INSTANCE_H_

#include <functional>
#include <optional>

// seerep-msgs
#include <seerep-msgs/label-with-instance.h>
// seerep-hdf5-core
#include <seerep-hdf5-core/hdf5-core-instance.h>

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

namespace seerep_core
{
/**
 * @brief This is the class for a single instance
 *
 * It handles all data regarding instances. The attributes of the instances are queriable.
 * And the attributes can also be written. The changes are persistet in the HDF5 file via the
 * corresponding IO class.
 *
 * The UUID of all data showing the instances are stored and can be easily queried.
 *
 * @todo check if core-instance and core-instances can be merge into one class like
 * it's done for image and point cloud
 */
class CoreInstance
{
public:
  /**
   * @brief Contructs the object based on the given UUID and the HDF5 IO object
   * @param hdf5_io a shared pointer to the object handling the HDF5 io for instances
   * @param uuid the UUID of this instance
   */
  CoreInstance(std::shared_ptr<seerep_hdf5_core::Hdf5CoreInstance> hdf5_io, const boost::uuids::uuid& uuidInstance);
  ~CoreInstance();

  /**
   * @brief Returns the UUID of the instance
   * @return UUID of the instance
   */
  boost::uuids::uuid getUUID();

  /**
   * @brief Returns the value of the attribute defined by the key
   * @param key the key of the attributes to get the value of it
   * @return optional string with the value of the attribute if it exists
   */
  std::optional<std::string> getAttribute(const std::string& key) const;
  /**
   * @brief Writes the value of the attribute defined by the key
   * @param key the key of the attributes
   * @param value the value to be written
   */
  void writeAttribute(const std::string& key, const std::string& value);

  /**
   * @brief Returns the UUIDs of the images showing this instance
   * @return Vector of the UUIDs of the images showings this instance
   */
  std::vector<boost::uuids::uuid> getImages() const;
  /**
   * @brief adds an image to this instance
   * @param uuidDataset the UUID of the image
   */
  void addImage(const boost::uuids::uuid& uuidDataset);

private:
  /**
   * @brief fills the member variables based on the HDF5 file
   */
  void recreateInstances();
  /** @brief shared pointer to the object handling the HDF5 io for instances */
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreInstance> m_hdf5_io;

  /** @brief the UUID of this instance */
  boost::uuids::uuid m_uuid;
  /** @brief the attributes of this instance */
  std::unordered_map<std::string, std::string> m_attributes;

  /** @brief vector of UUIDs of images showing this instance */
  std::vector<boost::uuids::uuid> m_images;
  /** @brief vector of UUIDs of point clouds showing this instance */
  std::vector<boost::uuids::uuid> m_pointClouds;
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_INSTANCE_H_
