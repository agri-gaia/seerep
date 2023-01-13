#ifndef SEEREP_HDF5_PY_HDF5_PY_H_
#define SEEREP_HDF5_PY_HDF5_PY_H_

// seerep-hdf5
#include "seerep-hdf5-py/hdf5-py-general.h"
#include "seerep-hdf5-py/hdf5-py-image.h"
#include "seerep-hdf5-py/hdf5-py-pointcloud.h"
#include "seerep-hdf5-py/hdf5-py-tf.h"

// pybind
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(seerephdf5py, m)
{
  m.doc() = "seerep hdf5 python bindings";

  py::class_<seerep_hdf5_py::Hdf5FileWrapper>(m, "File")
      .def(py::init<const std::string&>(), py::arg("filename"))
      .def("__repr__",
           [](seerep_hdf5_py::Hdf5FileWrapper& f) { return "<seerephdf5py.File '" + f.getFile()->getName() + "'>"; });

  py::class_<seerep_hdf5_py::Hdf5PyImage>(m, "ImageWriter")
      .def(py::init<seerep_hdf5_py::Hdf5FileWrapper&>(), py::arg("hdf5_file"));

  py::class_<seerep_hdf5_py::Hdf5PyPointCloud>(m, "PointCloudWriter")
      .def(py::init<seerep_hdf5_py::Hdf5FileWrapper&>(), py::arg("hdf5_file"));

  py::class_<seerep_hdf5_py::Hdf5PyTf>(m, "TfWriter")
      .def(py::init<seerep_hdf5_py::Hdf5FileWrapper&>(), py::arg("hdf5_file"));
}

#endif /* SEEREP_HDF5_PY_HDF5_PY_H_ */
