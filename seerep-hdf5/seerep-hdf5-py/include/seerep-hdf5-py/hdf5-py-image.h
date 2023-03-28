#ifndef SEEREP_HDF5_PY_HDF5_PY_IMAGE_H_
#define SEEREP_HDF5_PY_HDF5_PY_IMAGE_H_

// highfive
#include <highfive/H5File.hpp>

// seerep_hdf5
#include <seerep_hdf5_core/hdf5_core_image.h>

#include "seerep-hdf5-py/hdf5-py-general.h"

// seerep-msgs
#include <seerep_msgs/image.pb.h>

// std
#include <boost/geometry.hpp>
#include <optional>

// pybind
#include <pybind11_catkin/pybind11/numpy.h>
#include <pybind11_catkin/pybind11/pybind11.h>

namespace py = pybind11;

namespace seerep_hdf5_py
{

class Hdf5PyImage : public seerep_hdf5_core::Hdf5CoreImage, public Hdf5PyGeneral
{
public:
  Hdf5PyImage(Hdf5FileWrapper& hdf5_file);

  std::vector<std::string> getImages();

  void writeImage(const std::string& uuid, const std::string& frameId, int64_t seconds, int32_t nanos,
                  uint32_t sequence, const py::array& image, const std::vector<GeneralLabel>& generalLabels,
                  const std::vector<CategorizedBoundingBoxLabel<2>>& bbLabels);

  std::tuple<py::array, std::vector<GeneralLabel>, std::vector<CategorizedBoundingBoxLabel<2>>>
  readImage(const std::string& uuid);
};

}  // namespace seerep_hdf5_py

#endif /* SEEREP_HDF5_PY_HDF5_PY_IMAGE_H_ */
