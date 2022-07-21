#ifndef SEEREP_HDF5_CORE_HDF5_CORE_TF_H_
#define SEEREP_HDF5_CORE_HDF5_CORE_TF_H_

// highfive
#include <highfive/H5File.hpp>

// seerephdf5
#include "seerep-hdf5-core/hdf5-core-general.h"

// ros-msgs (tf)
#include <geometry_msgs/TransformStamped.h>

// std
#include <boost/geometry.hpp>
#include <optional>

namespace seerep_hdf5_core
{
class Hdf5CoreTf : public Hdf5CoreGeneral
{
public:
  Hdf5CoreTf(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  std::optional<std::vector<geometry_msgs::TransformStamped>> readTransformStamped(const std::string& id);
  std::optional<std::vector<std::string>> readTransformStampedFrames(const std::string& id);

private:
  std::string readFrame(const std::string& frameName, const std::shared_ptr<const HighFive::Group>& group_ptr) const;

  std::vector<std::vector<int64_t>> readTime(const std::string& hdf5DatasetTimePath) const;
  std::vector<std::vector<double>> readTranslation(const std::string& hdf5DatasetTransPath) const;
  std::vector<std::vector<double>> readRotation(const std::string& hdf5DatasetRotPath) const;

  std::optional<std::vector<geometry_msgs::TransformStamped>>
  convertToTfs(const long unsigned int& size, const std::string& parentframe, const std::string& childframe,
               const std::vector<std::vector<int64_t>>& time, const std::vector<std::vector<double>>& trans,
               const std::vector<std::vector<double>>& rot);

public:
  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_TF = "tf";

  inline static const std::string SIZE = "size";
};

}  // namespace seerep_hdf5_core

#endif /* SEEREP_HDF5_CORE_HDF5_CORE_TF_H_ */
