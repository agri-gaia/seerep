#ifndef SEEREP_HDF5_FB_IMAGE_H_
#define SEEREP_HDF5_FB_IMAGE_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include <seerep-hdf5-core/hdf5-core-image.h>

#include "seerep-hdf5-fb/hdf5-fb-general.h"

// seerep-msgs
#include <seerep-msgs/image_generated.h>

// seerep-com
#include <seerep-com/image_service.grpc.fb.h>

// std
#include <grpcpp/grpcpp.h>

#include <boost/geometry.hpp>
#include <optional>

#include "flatbuffers/grpc.h"

namespace seerep_hdf5_fb
{
// make nested flatbuffers readable
typedef flatbuffers::Vector<uint8_t> ByteArrayFb;

class Hdf5FbImage : public Hdf5FbGeneral, public seerep_hdf5_core::Hdf5CoreImage
{
public:
  Hdf5FbImage(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  void writeImage(const std::string& id, const seerep::fb::Image& image);
  void writeImageBoundingBox2DLabeled(const std::string& id,
                                      const seerep::fb::BoundingBoxes2DLabeledStamped& bb2dLabeledStamped);

  std::optional<flatbuffers::grpc::Message<seerep::fb::Image>> readImage(const std::string& id,
                                                                         const bool withoutData = false);
};

}  // namespace seerep_hdf5_fb

#endif /* SEEREP_HDF5_FB_IMAGE_H_ */
