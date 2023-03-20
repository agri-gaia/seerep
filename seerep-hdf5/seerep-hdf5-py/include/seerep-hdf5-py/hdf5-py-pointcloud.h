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
#include <pybind11_catkin/pybind11/numpy.h>
#include <pybind11_catkin/pybind11/pybind11.h>
#include <pybind11_catkin/pybind11/stl.h>

namespace py = pybind11;

namespace seerep_hdf5_py
{

class Hdf5PyPointCloud : public Hdf5PyGeneral
{
public:
  Hdf5PyPointCloud(Hdf5FileWrapper& hdf5File);

  std::vector<std::string> getPointClouds();

  void writePointCloud(const std::string& uuid, const std::string& frameId, int64_t seconds, int32_t nanos,
                       uint32_t sequence, const std::map<std::string, py::array> fields,
                       const std::vector<GeneralLabel>& generalLabels,
                       const std::vector<CategorizedBoundingBoxLabel<3>>& bbLabels);

  std::tuple<std::map<std::string, py::array>, std::vector<GeneralLabel>, std::vector<CategorizedBoundingBoxLabel<3>>>
  readPointCloud(const std::string& uuid);

private:
  template <typename T>
  bool checkType(const py::dtype& type);

  template <typename T>
  bool getFieldData(const std::vector<std::string>& fieldNames, const std::map<std::string, py::array>& fields,
                    std::vector<std::vector<std::vector<T>>>& fieldData);

  template <typename T, int Nfields>
  void getMinMax(const std::vector<std::vector<std::vector<T>>>& fieldData, std::array<T, Nfields>& min,
                 std::array<T, Nfields>& max);

  template <typename T, int Nfields>
  bool writeBoundingBox(const std::string& cloudGroupId, const std::vector<std::string>& fieldNames,
                        const std::map<std::string, py::array>& fields);

  void writeField(const std::string& cloudGroupId, const std::string& fieldName,
                  const std::map<std::string, py::array>& fields);

  template <typename T>
  bool writeFieldTyped(const std::string& cloudGroupId, const std::string& fieldDatasetId, const std::string& fieldName,
                       const std::map<std::string, py::array>& fields);

  template <typename T, typename Second, typename... Other>
  bool writeFieldTyped(const std::string& cloudGroupId, const std::string& fieldDatasetId, const std::string& fieldName,
                       const std::map<std::string, py::array>& fields);

  template <typename T>
  py::array readField(std::shared_ptr<HighFive::DataSet> fieldDatasetPtr);
};

} /* namespace seerep_hdf5_py */

#include "impl/hdf5-py-pointcloud.hpp"  // NOLINT

#endif /* SEEREP_HDF5_PY_HDF5_PY_POINT_CLOUD_H_ */
