#ifndef SEEREP_IO_PB_IO_PB_TF_H_
#define SEEREP_IO_PB_IO_PB_TF_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-io
#include "seerep-io-pb/io-pb-general.h"

// seerep-msgs
#include <seerep-msgs/transform_stamped.pb.h>

// std
#include <optional>

#include <boost/geometry.hpp>

namespace seerep_io_pb
{
class IoPbTf : public IoPbGeneral
{
public:
  IoPbTf(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  void writeTransformStamped(const seerep::TransformStamped& tf);

  std::optional<std::vector<seerep::TransformStamped>> readTransformStamped(const std::string& id);
  std::optional<std::vector<std::string>> readTransformStampedFrames(const std::string& id);

private:
  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_TF = "tf";

  inline static const std::string SIZE = "size";
};

} /* namespace seerep_io_pb */

#endif /* SEEREP_IO_PB_IO_PB_TF_H_ */
