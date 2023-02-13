#ifndef SEEREP_HDF5_PY_HDF5_PY_POINT_CLOUD_H_
#define SEEREP_HDF5_PY_HDF5_PY_POINT_CLOUD_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include "seerep-hdf5-core/hdf5-core-point-cloud.h"
#include "seerep-hdf5-py/hdf5-py-general.h"

// seerep-msgs
#include "seerep-msgs/point_cloud_2.pb.h"

// std
#include <boost/geometry.hpp>
#include <optional>

// pybind
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace seerep_hdf5_py
{

class Hdf5PyPointCloud : public Hdf5PyGeneral
{
public:
  Hdf5PyPointCloud(Hdf5FileWrapper& hdf5_file);

  std::vector<std::string> getPointClouds();

  void writePointCloud(const std::string& uuid, const std::string& frame_id, int64_t seconds, int32_t nanos,
                       uint32_t sequence, const std::map<std::string, py::array> channels,
                       const std::vector<GeneralLabel>& general_labels,
                       const std::vector<CategorizedBoundingBoxLabel<3>>& bb_labels);

  std::map<std::string, py::array> readPointCloud(const std::string& uuid);

private:
  template <typename T>
  bool checkType(const py::dtype& type);

  template <typename T>
  bool getChannelData(const std::vector<std::string>& channel_names, const std::map<std::string, py::array>& channels,
                      std::vector<std::vector<std::vector<T>>>& channel_data);

  template <typename T, int Nchannels>
  void getMinMax(const std::vector<std::vector<std::vector<T>>>& data, std::array<T, Nchannels>& min,
                 std::array<T, Nchannels>& max);

  void writeChannel(const std::string& cloud_group_id, const std::string& channel_name,
                    const std::map<std::string, py::array>& channels);

  template <typename T>
  bool writeChannelTyped(const std::string& cloud_group_id, const std::string& channel_dataset_id,
                         const std::vector<std::vector<std::string>>& channel_names,
                         const std::map<std::string, py::array>& channels);

  template <typename T, typename Second, typename... Other>
  bool writeChannelTyped(const std::string& cloud_group_id, const std::string& channel_dataset_id,
                         const std::vector<std::vector<std::string>>& channel_names,
                         const std::map<std::string, py::array>& channels);

  template <typename T>
  py::array readField(std::shared_ptr<HighFive::DataSet> field_dataset);
};

} /* namespace seerep_hdf5_py */

#include "impl/hdf5-py-pointcloud.hpp"  // NOLINT

#endif /* SEEREP_HDF5_PY_HDF5_PY_POINT_CLOUD_H_ */
