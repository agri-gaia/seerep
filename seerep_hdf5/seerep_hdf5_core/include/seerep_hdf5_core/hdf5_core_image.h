#ifndef SEEREP_HDF5_CORE_HDF5_CORE_IMAGE_H_
#define SEEREP_HDF5_CORE_HDF5_CORE_IMAGE_H_

// highfive
#include <highfive/H5File.hpp>

// seerep_hdf5_core
#include "hdf5_core_cameraintrinsics.h"
#include "hdf5_core_datatype_interface.h"
#include "hdf5_core_general.h"

// seerep_msgs
#include <seerep_msgs/dataset_indexable.h>
#include <seerep_msgs/labels_with_instance_with_category.h>

// std
#include <boost/geometry.hpp>
#include <optional>

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

namespace seerep_hdf5_core
{
/**
 * @brief Helper struct to summarize the general attributes of an image
 *
 */
struct ImageAttributes
{
  uint32_t height;
  uint32_t width;
  uint32_t step;
  std::string encoding;
  bool isBigendian;
  std::string cameraIntrinsicsUuid;
};

/**
 * @brief This class encompasses all hdf5-io functions which are message type independent
 *
 * This means that the functions can currently be used for flatbuffers and protocol buffers
 *
 */
class Hdf5CoreImage : public virtual Hdf5CoreGeneral, public Hdf5CoreDatatypeInterface
{
public:
  /**
   * @brief Construct a new Hdf 5 Core Image object
   *
   * @param file
   * @param write_mtx
   */
  Hdf5CoreImage(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  /**
   * @brief Get the indices for the seerep core from an image data group
   *
   * @param uuid the uuid of the image data group
   * @return std::optional<seerep_core_msgs::DatasetIndexable> the seerep core message with the indices
   */
  std::optional<seerep_core_msgs::DatasetIndexable> readDataset(const boost::uuids::uuid& uuid);

  /**
   * @brief Get the indices for the seerep core from an image data group
   *
   * Used when recreating the indices on a server restart
   *
   * @param uuid the uuid of the image data group
   * @return std::optional<seerep_core_msgs::DatasetIndexable> the seerep core message with the indices
   */
  std::optional<seerep_core_msgs::DatasetIndexable> readDataset(const std::string& uuid);

  /**
   * @brief Get all of the data set UUIDs in an image data group
   *
   * @return std::vector<std::string> vector of UUIDs (strings)
   */
  std::vector<std::string> getDatasetUuids();

  /**
   * @brief Write generals labels based on C++ data structures to HdF5
   *
   * @param uuid uuid of the image data group
   * @param labelsWithInstanceWithCategory vector of labels with instances in multiple categories
   */
  void writeLabelsGeneral(
      const std::string& uuid,
      const std::vector<seerep_core_msgs::LabelsWithInstanceWithCategory>& labelsWithInstanceWithCategory);

  /**
   * @brief Write the general attributes of an image to hdf5
   *
   * @param id uuid of the image data group
   * @param attributes struct with the general attributes
   */
  void writeImageAttributes(const std::string& id, const ImageAttributes& attributes);

  /**
   * @brief Read general attributes of an image from hdf5
   *
   * @param id uuid of the image data group
   * @return ImageAttributes struct with the general attributes
   */
  ImageAttributes readImageAttributes(const std::string& id);

  /**
   * @brief Get the path to the hdf5 group of an image
   *
   * @param id uuid of the image data group
   * @return const std::string the path to the image group
   */
  const std::string getHdf5GroupPath(const std::string& id) const;

  /**
   * @brief Get the path to the image raw data dataset based on the data group uuid
   *
   * @param id uuid of the image data group
   * @return const std::string path to the image dataset
   */
  const std::string getHdf5DataSetPath(const std::string& id) const;
  /**
   * @brief fetch the camera intrinsics using the provided uuid, compute the frustrum of the camera and apply them to
   * the provided bounding box
   *
   * @param camintrinsics_uuid uuid of camera intrinsics to be fetched
   * @param bb boundingbox in which the computed frustrum has to be saved
   */
  void computeFrusturmBB(const std::string& camintrinsics_uuid, seerep_core_msgs::AABB& bb);

public:
  // image attribute keys
  inline static const std::string HEIGHT = "height";
  inline static const std::string WIDTH = "width";
  inline static const std::string ENCODING = "encoding";
  inline static const std::string IS_BIGENDIAN = "is_bigendian";
  inline static const std::string STEP = "step";
  inline static const std::string IS_DENSE = "is_dense";
  inline static const std::string CAMERA_INTRINSICS_UUID = "camera_intrinsics_uuid";

  inline static const std::string RAWDATA = "rawdata";

  // datatype group name in hdf5
  inline static const std::string HDF5_GROUP_IMAGE = "images";

private:
  /** @brief object handling the HDF5 file IO regarding CIs */
  std::shared_ptr<seerep_hdf5_core::Hdf5CoreCameraIntrinsics> m_ioCI;
};

}  // namespace seerep_hdf5_core

#endif /* SEEREP_HDF5_CORE_HDF5_CORE_IMAGE_H_ */
