#ifndef SEEREP_HDF5_PY_HDF5_PY_H_
#define SEEREP_HDF5_PY_HDF5_PY_H_

// seerep-hdf5
#include "seerep-hdf5-py/hdf5-py-general.h"

// pybind
#include <pybind11/pybind11.h>

int add(int i, int j)
{
  return i + j;
}

namespace py = pybind11;

PYBIND11_MODULE(seerephdf5py, m)
{
  m.doc() = "seerep hdf5 python bindings";

  py::class_<seerep_hdf5_py::Hdf5FileWrapper>(m, "File")
      .def(py::init<const std::string&>(), py::arg("filename"))
      .def("__repr__", [](const seerep_hdf5_py::Hdf5FileWrapper& f) {
        return "<seerephdf5py.File '" + f.getFile()->getName() + "'>";
      });
}

#endif /* SEEREP_HDF5_PY_HDF5_PY_H_ */
