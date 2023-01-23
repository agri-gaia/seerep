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
class Hdf5CoreTf : public virtual Hdf5CoreGeneral
{
public:
  Hdf5CoreTf(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  std::optional<std::vector<geometry_msgs::TransformStamped>> readTransformStamped(const std::string& id);
  std::optional<std::vector<std::string>> readTransformStampedFrames(const std::string& id);

  std::string readFrame(const std::string& frameName, const std::shared_ptr<const HighFive::Group>& group_ptr) const;

  void writeTimestamp(const std::string& datasetPath, std::array<int64_t, 2> timestamp);
  std::vector<std::vector<int64_t>> readTimestamps(const std::string& timeDatasetPath) const;

  void writeTranslation(const std::string& translationDatasetPath, std::array<double, 3> translation);
  std::vector<std::vector<double>> readTranslations(const std::string& translationDatasetPath) const;

  void writeRotation(const std::string& rotationsDatasetPath, std::array<double, 4> rotation);
  std::vector<std::vector<double>> readRotations(const std::string& rotationDatasetPath) const;

private:
  std::optional<std::vector<geometry_msgs::TransformStamped>>
  convertToTfs(const long unsigned int& size, const std::string& parentframe, const std::string& childframe,
               const std::vector<std::vector<int64_t>>& timestamps,
               const std::vector<std::vector<double>>& translations, const std::vector<std::vector<double>>& rotations);

public:
  inline static const std::string HDF5_GROUP_TF = "tf";
  inline static const std::string SIZE = "size";

  inline static const std::string HDF5_DATASET_TIMESTAMPS = "/time";
  inline static const std::string HDF5_DATASET_TRANSLATION = "/translation";
  inline static const std::string HDF5_DATASET_ROTATION = "/rotation";
};

}  // namespace seerep_hdf5_core

#endif /* SEEREP_HDF5_CORE_HDF5_CORE_TF_H_ */
