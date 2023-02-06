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

  // Label Types
  py::class_<seerep_hdf5_py::InstanceLabel>(m, "InstanceLabel")
      .def(py::init<const std::string&, const std::string&>(), py::arg("label"), py::arg("instance_uuid"))
      .def_readwrite("label", &seerep_hdf5_py::InstanceLabel::label)
      .def_readwrite("instance_uuid", &seerep_hdf5_py::InstanceLabel::instance_uuid)
      .def("__repr__", [](seerep_hdf5_py::InstanceLabel& l) {
        return "<seerephdf5py.InstanceLabel '" + l.label + "', '" + l.instance_uuid + "'>";
      });

  py::class_<seerep_hdf5_py::GeneralLabel>(m, "GeneralLabel")
      .def(py::init<const std::string&>(), py::arg("category"))
      .def_readwrite("category", &seerep_hdf5_py::GeneralLabel::category)
      .def_readwrite("labels", &seerep_hdf5_py::GeneralLabel::labels)
      .def("addLabel", &seerep_hdf5_py::GeneralLabel::addLabel, py::arg("label"))
      .def("__repr__",
           [](seerep_hdf5_py::GeneralLabel& l) { return "<seerephdf5py.GeneralLabel '" + l.category + "'>"; });

  py::class_<seerep_hdf5_py::BoundingBoxLabel<3>>(m, "BoundingBoxLabel3D")
      .def(py::init<seerep_hdf5_py::InstanceLabel&, std::array<double, 3>&, std::array<double, 3>&>(), py::arg("label"),
           py::arg("min_point"), py::arg("max_point"))
      .def_readwrite("label", &seerep_hdf5_py::BoundingBoxLabel<3>::label)
      .def_readwrite("min_point", &seerep_hdf5_py::BoundingBoxLabel<3>::max_point)
      .def_readwrite("max_point", &seerep_hdf5_py::BoundingBoxLabel<3>::max_point)
      .def("__repr__", [](seerep_hdf5_py::BoundingBoxLabel<3>& l) {
        return "<seerephdf5py.BoundingBoxLabel3D min(" + std::to_string(l.min_point[0]) + ", " +
               std::to_string(l.min_point[1]) + ", " + std::to_string(l.min_point[2]) + "), max(" +
               std::to_string(l.max_point[0]) + ", " + std::to_string(l.max_point[1]) + ", " +
               std::to_string(l.max_point[2]) + ")>";
      });

  py::class_<seerep_hdf5_py::BoundingBoxLabel<2>>(m, "BoundingBoxLabel2D")
      .def(py::init<seerep_hdf5_py::InstanceLabel&, std::array<double, 2>&, std::array<double, 2>&>(), py::arg("label"),
           py::arg("min_point"), py::arg("max_point"))
      .def_readwrite("label", &seerep_hdf5_py::BoundingBoxLabel<2>::label)
      .def_readwrite("min_point", &seerep_hdf5_py::BoundingBoxLabel<2>::max_point)
      .def_readwrite("max_point", &seerep_hdf5_py::BoundingBoxLabel<2>::max_point)
      .def("__repr__", [](seerep_hdf5_py::BoundingBoxLabel<2>& l) {
        return "<seerephdf5py.BoundingBoxLabel2D min(" + std::to_string(l.min_point[0]) + ", " +
               std::to_string(l.min_point[1]) + "), max(" + std::to_string(l.max_point[0]) + ", " +
               std::to_string(l.max_point[1]) + ")>";
      });

  py::class_<seerep_hdf5_py::CategorizedBoundingBoxLabel<3>>(m, "CategorizedBoundingBoxLabel3D")
      .def(py::init<const std::string&>(), py::arg("category"))
      .def_readwrite("category", &seerep_hdf5_py::CategorizedBoundingBoxLabel<3>::category)
      .def_readwrite("labels", &seerep_hdf5_py::CategorizedBoundingBoxLabel<3>::labels)
      .def("addLabel", &seerep_hdf5_py::CategorizedBoundingBoxLabel<3>::addLabel, py::arg("label"))
      .def("__repr__", [](seerep_hdf5_py::CategorizedBoundingBoxLabel<3>& l) {
        return "<seerephdf5py.CategorizedBoundingBoxLabel3D '" + l.category + "'>";
      });

  py::class_<seerep_hdf5_py::CategorizedBoundingBoxLabel<2>>(m, "CategorizedBoundingBoxLabel2D")
      .def(py::init<const std::string&>(), py::arg("category"))
      .def_readwrite("category", &seerep_hdf5_py::CategorizedBoundingBoxLabel<2>::category)
      .def_readwrite("labels", &seerep_hdf5_py::CategorizedBoundingBoxLabel<2>::labels)
      .def("addLabel", &seerep_hdf5_py::CategorizedBoundingBoxLabel<2>::addLabel, py::arg("label"))
      .def("__repr__", [](seerep_hdf5_py::CategorizedBoundingBoxLabel<2>& l) {
        return "<seerephdf5py.CategorizedBoundingBoxLabel2D '" + l.category + "'>";
      });

  // IO
  py::class_<seerep_hdf5_py::Hdf5FileWrapper>(m, "File")
      .def(py::init<const std::string&>(), py::arg("filename"))
      .def("__repr__",
           [](seerep_hdf5_py::Hdf5FileWrapper& f) { return "<seerephdf5py.File '" + f.getFile()->getName() + "'>"; });

  py::class_<seerep_hdf5_py::Hdf5PyImage>(m, "ImageIO")
      .def(py::init<seerep_hdf5_py::Hdf5FileWrapper&>(), py::arg("hdf5_file"))
      .def("writeImage", &seerep_hdf5_py::Hdf5PyImage::writeImage, py::arg("uuid"), py::arg("frame_id"),
           py::arg("seconds"), py::arg("nanos"), py::arg("sequence"), py::arg("image"),
           py::arg("general_labels") = std::vector<seerep_hdf5_py::GeneralLabel>(),
           py::arg("bb_labels") = std::vector<seerep_hdf5_py::CategorizedBoundingBoxLabel<2>>())
      .def("readImage", &seerep_hdf5_py::Hdf5PyImage::readImage, py::arg("uuid"));

  py::class_<seerep_hdf5_py::Hdf5PyPointCloud>(m, "PointCloudIO")
      .def(py::init<seerep_hdf5_py::Hdf5FileWrapper&>(), py::arg("hdf5_file"))
      .def("getPointClouds", &seerep_hdf5_py::Hdf5PyPointCloud::getPointClouds)
      .def("writePointCloud", &seerep_hdf5_py::Hdf5PyPointCloud::writePointCloud, py::arg("uuid"), py::arg("frame_id"),
           py::arg("seconds"), py::arg("nanos"), py::arg("sequence"), py::arg("channels"))
      .def("readPointCloud", &seerep_hdf5_py::Hdf5PyPointCloud::readPointCloud, py::arg("uuid"));

  py::class_<seerep_hdf5_py::Hdf5PyTf>(m, "TfIO").def(py::init<seerep_hdf5_py::Hdf5FileWrapper&>(),
                                                      py::arg("hdf5_file"));
}

#endif /* SEEREP_HDF5_PY_HDF5_PY_H_ */
