#ifndef SEEREP_HDF5_CORE_HDF5_CORE_TF_H_
#define SEEREP_HDF5_CORE_HDF5_CORE_TF_H_

// highfive
#include <highfive/H5File.hpp>

// seerep_hdf5_core
#include "hdf5_core_general.h"

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
  Hdf5CoreTf(std::shared_ptr<HighFive::File>& file,
             std::shared_ptr<std::mutex>& write_mtx);

  std::optional<std::vector<geometry_msgs::TransformStamped>>
  readTransformStamped(const std::string& id, const bool isStatic);
  std::optional<std::vector<std::string>>
  readTransformStampedFrames(const std::string& id, const bool isStatic);

private:
  std::vector<std::vector<int64_t>>
  readTime(const std::string& hdf5DatasetTimePath) const;
  std::vector<std::vector<double>>
  readTranslation(const std::string& hdf5DatasetTransPath) const;
  std::vector<std::vector<double>>
  readRotation(const std::string& hdf5DatasetRotPath) const;

  std::optional<std::vector<geometry_msgs::TransformStamped>>
  convertToTfs(const long unsigned int& size, const std::string& parentframe,
               const std::string& childframe,
               const std::vector<std::vector<int64_t>>& time,
               const std::vector<std::vector<double>>& trans,
               const std::vector<std::vector<double>>& rot);

public:
  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_TF = "tf";
  inline static const std::string HDF5_GROUP_TF_STATIC = "tf_static";

  inline static const std::string SIZE = "size";

  inline static const std::string PARENT_FRAME = "PARENT_FRAME";
  inline static const std::string CHILD_FRAME = "CHILD_FRAME";
};

}  // namespace seerep_hdf5_core

#endif /* SEEREP_HDF5_CORE_HDF5_CORE_TF_H_ */
