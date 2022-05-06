#ifndef SEEREP_CORE_CORE_INSTANCES_H_
#define SEEREP_CORE_CORE_INSTANCES_H_

#include <functional>
#include <optional>

#include "core-instance.h"

// seerep-msgs
#include <seerep-msgs/datatype.h>
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
 * @brief This is the class for a all instances
 *
 * It handles all data regarding instances. New instances can be created. The data
 * of existing instances can be queried or modified.
 *
 * @todo check if core-instance and core-instances can be merge into one class like
 * it's done for image and point cloud
 */
class CoreInstances
{
public:
  /**
   * @brief Contructs the object and creates the instances using the HDF5 IO object
   * @param hdf5_io a shared pointer to the object handling the HDF5 io for instances
   */
  CoreInstances(std::shared_ptr<seerep_hdf5_core::Hdf5CoreInstance> hdf5_io);
  ~CoreInstances();

  /**
   * @brief Create a new instance with a random UUID
   * @param label the label for the instance
   * @return shared pointer to the created instance object
   */
  std::shared_ptr<seerep_core::CoreInstance> createNewInstance(const std::string& label);
  /**
   * @brief Create a new instance with a given UUID and label
   * @param labelWithInstance the label and the UUID of the new instance
   * @return shared pointer to the created instance object
   */
  std::shared_ptr<seerep_core::CoreInstance>
  createNewInstance(const seerep_core_msgs::LabelWithInstance& labelWithInstance);

  /**
   * @brief Returns the value of the attribute of the instance with the defined UUID defined by the key
   * @param uuidInstance the UUID of the targeted instance
   * @param key the key of the attributes to get the value of it
   * @return optional string with the value of the attribute if it exists
   */
  std::optional<std::string> getAttribute(const boost::uuids::uuid& uuidInstance, const std::string& key) const;
  /**
   * @brief Writes the value of the attribute of the instance with the defined UUID  defined by the key
   * @param uuidInstance the UUID of the targeted instance
   * @param key the key of the attributes
   * @param value the value to be written
   */
  void writeAttribute(const boost::uuids::uuid& uuidInstance, const std::string& key, const std::string& value);

  /**
   * @brief Returns the UUIDs of the images showing these instances
   * @param instanceIds vector of UUIDs of the targeted instances
   * @param dataset the datatype to be returned
   * @return Vector of the UUIDs of the images showings these instances
   */
  std::vector<boost::uuids::uuid> getDatasets(const std::vector<boost::uuids::uuid>& instanceIds,
                                              const seerep_core_msgs::Datatype& datatype) const;
  /**
   * @brief adds an image to this instance
   * @param labelWithInstance the UUID of the targeted instance and the assigned label
   * @param uuidDataset the UUID of the image
   * @param dataset the type of the dataset
   */
  void addDataset(const seerep_core_msgs::LabelWithInstance& labelWithInstance, const boost::uuids::uuid& uuidDataset,
                  const seerep_core_msgs::Datatype& datatype);

private:
  /**
   * @brief fills the member variables based on the HDF5 file. It creates the instance objects
   */
  void recreateInstances();
  /** @brief shared pointer to the object handling the HDF5 io for instances */
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreInstance> m_hdf5_io;
  /** @brief map from the uuid of an instance to the object of it */
  std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_core::CoreInstance>, boost::hash<boost::uuids::uuid>>
      m_instances;

public:
  inline static const std::string ATTRIBUTELABEL = "label";
};

} /* namespace seerep_core */

#endif  // SEEREP_CORE_CORE_INSTANCES_H_
