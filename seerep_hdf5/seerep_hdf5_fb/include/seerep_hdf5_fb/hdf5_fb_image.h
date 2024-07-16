#ifndef SEEREP_HDF5_FB_IMAGE_H_
#define SEEREP_HDF5_FB_IMAGE_H_

// highfive
#include <highfive/H5File.hpp>

// seerep_hdf5
#include <seerep_hdf5_core/hdf5_core_image.h>

#include "hdf5_fb_general.h"

// seerep-msgs
#include <seerep_msgs/image_generated.h>

// seerep-com
#include <seerep_com/image_service.grpc.fb.h>

// std
#include <flatbuffers/grpc.h>
#include <grpcpp/grpcpp.h>

#include <boost/geometry.hpp>
#include <optional>

namespace seerep_hdf5_fb
{
// make nested flatbuffers readable
typedef flatbuffers::Vector<uint8_t> ByteArrayFb;

/**
 * @brief The class Hdf5FbImage is used to write and read flatbuffers image messages to/from hdf5 files
 *
 */
class Hdf5FbImage : public Hdf5FbGeneral, public seerep_hdf5_core::Hdf5CoreImage
{
public:
  /**
   * @brief Construct a new Hdf5FbImage object
   *
   * @param file shared pointer to the hdf5 file, to write the images to
   * @param write_mtx mutex to ensure thread safety
   */
  Hdf5FbImage(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  /**
   * @brief Write a flatbuffers image message to hdf5
   *
   * @param id
   * @param image the flatbuffers image message
   */
  void writeImage(const std::string& id, const seerep::fb::Image& image);

  /**
   * @brief Read an image from hdf5 and receive it as a flatbuffers message
   *
   * @param id the uuid of the image data group
   * @param withoutData should the image data be excluded from the message?
   * @return std::optional<flatbuffers::grpc::Message<seerep::fb::Image>> the flatbuffers image message, can be empty if
   * an error occurred
   */
  std::optional<flatbuffers::grpc::Message<seerep::fb::Image>> readImage(const std::string& id,
                                                                         const bool withoutData = false);
};

}  // namespace seerep_hdf5_fb

#endif /* SEEREP_HDF5_FB_IMAGE_H_ */
