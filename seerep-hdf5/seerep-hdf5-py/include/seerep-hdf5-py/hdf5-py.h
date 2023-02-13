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

// define python module
PYBIND11_MODULE(seerephdf5py, m)
{
  m.doc() = "seerep hdf5 python bindings";

  // Label Types
  py::class_<seerep_hdf5_py::InstanceLabel>(m, "InstanceLabel")
      .def(py::init<const std::string&, const std::string&>(), py::arg("label"), py::arg("instance_uuid"))
      .def_readwrite("label", &seerep_hdf5_py::InstanceLabel::label_)
      .def_readwrite("instance_uuid", &seerep_hdf5_py::InstanceLabel::instanceUuid_)
      .def("__repr__", [](seerep_hdf5_py::InstanceLabel& l) {
        return "<seerephdf5py.InstanceLabel '" + l.label_ + "', '" + l.instanceUuid_ + "'>";
      });

  py::class_<seerep_hdf5_py::GeneralLabel>(m, "GeneralLabel")
      .def(py::init<const std::string&>(), py::arg("category"))
      .def_readwrite("category", &seerep_hdf5_py::GeneralLabel::category_)
      .def_readwrite("labels", &seerep_hdf5_py::GeneralLabel::labels_)
      .def("addLabel", &seerep_hdf5_py::GeneralLabel::addLabel, py::arg("label"))
      .def("__repr__",
           [](seerep_hdf5_py::GeneralLabel& l) { return "<seerephdf5py.GeneralLabel '" + l.category_ + "'>"; });

  py::class_<seerep_hdf5_py::BoundingBoxLabel<3>>(m, "BoundingBoxLabel3D")
      .def(py::init<seerep_hdf5_py::InstanceLabel&, std::array<double, 3>&, std::array<double, 3>&>(), py::arg("label"),
           py::arg("min_point"), py::arg("max_point"))
      .def_readwrite("label", &seerep_hdf5_py::BoundingBoxLabel<3>::label_)
      .def_readwrite("min_point", &seerep_hdf5_py::BoundingBoxLabel<3>::minPoint_)
      .def_readwrite("max_point", &seerep_hdf5_py::BoundingBoxLabel<3>::maxPoint_)
      .def("__repr__", [](seerep_hdf5_py::BoundingBoxLabel<3>& l) {
        return "<seerephdf5py.BoundingBoxLabel3D min(" + std::to_string(l.minPoint_[0]) + ", " +
               std::to_string(l.minPoint_[1]) + ", " + std::to_string(l.minPoint_[2]) + "), max(" +
               std::to_string(l.maxPoint_[0]) + ", " + std::to_string(l.maxPoint_[1]) + ", " +
               std::to_string(l.maxPoint_[2]) + ")>";
      });

  py::class_<seerep_hdf5_py::BoundingBoxLabel<2>>(m, "BoundingBoxLabel2D")
      .def(py::init<seerep_hdf5_py::InstanceLabel&, std::array<double, 2>&, std::array<double, 2>&>(), py::arg("label"),
           py::arg("min_point"), py::arg("max_point"))
      .def_readwrite("label", &seerep_hdf5_py::BoundingBoxLabel<2>::label_)
      .def_readwrite("min_point", &seerep_hdf5_py::BoundingBoxLabel<2>::minPoint_)
      .def_readwrite("max_point", &seerep_hdf5_py::BoundingBoxLabel<2>::maxPoint_)
      .def("__repr__", [](seerep_hdf5_py::BoundingBoxLabel<2>& l) {
        return "<seerephdf5py.BoundingBoxLabel2D min(" + std::to_string(l.minPoint_[0]) + ", " +
               std::to_string(l.minPoint_[1]) + "), max(" + std::to_string(l.maxPoint_[0]) + ", " +
               std::to_string(l.maxPoint_[1]) + ")>";
      });

  py::class_<seerep_hdf5_py::CategorizedBoundingBoxLabel<3>>(m, "CategorizedBoundingBoxLabel3D")
      .def(py::init<const std::string&>(), py::arg("category"))
      .def_readwrite("category", &seerep_hdf5_py::CategorizedBoundingBoxLabel<3>::category_)
      .def_readwrite("labels", &seerep_hdf5_py::CategorizedBoundingBoxLabel<3>::labels_)
      .def("addLabel", &seerep_hdf5_py::CategorizedBoundingBoxLabel<3>::addLabel, py::arg("label"))
      .def("__repr__", [](seerep_hdf5_py::CategorizedBoundingBoxLabel<3>& l) {
        return "<seerephdf5py.CategorizedBoundingBoxLabel3D '" + l.category_ + "'>";
      });

  py::class_<seerep_hdf5_py::CategorizedBoundingBoxLabel<2>>(m, "CategorizedBoundingBoxLabel2D")
      .def(py::init<const std::string&>(), py::arg("category"))
      .def_readwrite("category", &seerep_hdf5_py::CategorizedBoundingBoxLabel<2>::category_)
      .def_readwrite("labels", &seerep_hdf5_py::CategorizedBoundingBoxLabel<2>::labels_)
      .def("addLabel", &seerep_hdf5_py::CategorizedBoundingBoxLabel<2>::addLabel, py::arg("label"))
      .def("__repr__", [](seerep_hdf5_py::CategorizedBoundingBoxLabel<2>& l) {
        return "<seerephdf5py.CategorizedBoundingBoxLabel2D '" + l.category_ + "'>";
      });

  // Tf Types
  py::class_<seerep_hdf5_py::TfTransform>(m, "TfTransform")
      .def(py::init<>())
      .def(py::init<int64_t, int32_t, const std::string&, const std::string&, const std::array<double, 3>&,
                    const std::array<double, 4>&>(),
           py::arg("seconds"), py::arg("nanos"), py::arg("frame_id"), py::arg("child_frame_id"), py::arg("translation"),
           py::arg("rotation"))
      .def_readwrite("seconds", &seerep_hdf5_py::TfTransform::seconds_)
      .def_readwrite("nanos", &seerep_hdf5_py::TfTransform::nanos_)
      .def_readwrite("frame_id", &seerep_hdf5_py::TfTransform::frameId_)
      .def_readwrite("child_frame_id", &seerep_hdf5_py::TfTransform::childFrameId_)
      .def_readwrite("translation", &seerep_hdf5_py::TfTransform::translation_)
      .def_readwrite("rotation", &seerep_hdf5_py::TfTransform::rotation_)
      .def("__repr__", [](seerep_hdf5_py::TfTransform& l) {
        return "<seerephdf5py.TfTransform '" + l.frameId_ + "' '" + l.childFrameId_ + "'>";
      });

  // IO
  py::class_<seerep_hdf5_py::Hdf5FileWrapper>(m, "File")
      .def(py::init<const std::string&>(), py::arg("filename"))
      .def("createProject", &seerep_hdf5_py::Hdf5FileWrapper::createProject, py::arg("project_name"),
           py::arg("root_frame_id"))
      .def("setProjectGeolocation", &seerep_hdf5_py::Hdf5FileWrapper::setProjectGeolocation,
           py::arg("coordinate_system"), py::arg("ellipsoid"), py::arg("latitude"), py::arg("longitude"),
           py::arg("altitude"))
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
           py::arg("seconds"), py::arg("nanos"), py::arg("sequence"), py::arg("fields"),
           py::arg("general_labels") = std::vector<seerep_hdf5_py::GeneralLabel>(),
           py::arg("bb_labels") = std::vector<seerep_hdf5_py::CategorizedBoundingBoxLabel<3>>())
      .def("readPointCloud", &seerep_hdf5_py::Hdf5PyPointCloud::readPointCloud, py::arg("uuid"));

  py::class_<seerep_hdf5_py::Hdf5PyTf>(m, "TfIO")
      .def(py::init<seerep_hdf5_py::Hdf5FileWrapper&>(), py::arg("hdf5_file"))
      .def("writeTransform", &seerep_hdf5_py::Hdf5PyTf::writeTransformStamped, py::arg("tf"))
      .def("readTransform", &seerep_hdf5_py::Hdf5PyTf::readTransformStamped, py::arg("frame_id"));
}

#endif /* SEEREP_HDF5_PY_HDF5_PY_H_ */
