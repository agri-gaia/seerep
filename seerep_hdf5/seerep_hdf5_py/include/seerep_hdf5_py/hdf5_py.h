#ifndef SEEREP_HDF5_PY_HDF5_PY_H_
#define SEEREP_HDF5_PY_HDF5_PY_H_

// seerep_hdf5
#include "hdf5_py_general.h"
#include "hdf5_py_image.h"
#include "hdf5_py_pointcloud.h"
#include "hdf5_py_tf.h"

// pybind
#include <pybind11_catkin/pybind11/pybind11.h>

namespace py = pybind11;

// define python module
PYBIND11_MODULE(seerep_hdf5_py, m)
{
  m.doc() = "seerep hdf5 python bindings";

  // Label Types
  py::class_<seerep_hdf5_py::InstanceLabel>(m, "InstanceLabel")
      .def(py::init<const std::string&, float, const std::string&>(), py::arg("label"), py::arg("confidence"),
           py::arg("instance_uuid"))
      .def_readwrite("label", &seerep_hdf5_py::InstanceLabel::label_)
      .def_readwrite("confidence", &seerep_hdf5_py::InstanceLabel::confidence_)
      .def_readwrite("instance_uuid", &seerep_hdf5_py::InstanceLabel::instanceUuid_)
      .def("__repr__", [](seerep_hdf5_py::InstanceLabel& l) {
        return "<seerep_hdf5_py.InstanceLabel '" + l.label_ + "', '" + std::to_string(l.confidence_) + "', '" +
               l.instanceUuid_ + "'>";
      });

  py::class_<seerep_hdf5_py::GeneralLabel>(m, "GeneralLabel")
      .def(py::init<const std::string&>(), py::arg("category"))
      .def_readwrite("category", &seerep_hdf5_py::GeneralLabel::category_)
      .def_readwrite("labels", &seerep_hdf5_py::GeneralLabel::labels_)
      .def("addLabel", &seerep_hdf5_py::GeneralLabel::addLabel, py::arg("label"))
      .def("__repr__",
           [](seerep_hdf5_py::GeneralLabel& l) { return "<seerep_hdf5_py.GeneralLabel '" + l.category_ + "'>"; });

  py::class_<seerep_hdf5_py::BoundingBoxLabel<3>>(m, "BoundingBoxLabel3D")
      .def(py::init<seerep_hdf5_py::InstanceLabel&, std::array<double, 3>&, std::array<double, 3>&,
                    std::array<double, 4>&>(),
           py::arg("label"), py::arg("center"), py::arg("spatial_extent"), py::arg("rotation"))
      .def_readwrite("label", &seerep_hdf5_py::BoundingBoxLabel<3>::label_)
      .def_readwrite("center", &seerep_hdf5_py::BoundingBoxLabel<3>::bbCenter_)
      .def_readwrite("spatial_extent", &seerep_hdf5_py::BoundingBoxLabel<3>::bbSpatialExtent_)
      .def_readwrite("rotation", &seerep_hdf5_py::BoundingBoxLabel<3>::bbRotation_)
      .def("__repr__", [](seerep_hdf5_py::BoundingBoxLabel<3>& l) {
        return "<seerep_hdf5_py.BoundingBoxLabel3D center(" + std::to_string(l.bbCenter_[0]) + ", " +
               std::to_string(l.bbCenter_[1]) + ", " + std::to_string(l.bbCenter_[2]) + "), extent(" +
               std::to_string(l.bbSpatialExtent_[0]) + ", " + std::to_string(l.bbSpatialExtent_[1]) + ", " +
               std::to_string(l.bbSpatialExtent_[2]) + "), rotation(" + std::to_string(l.bbRotation_[0]) + ", " +
               std::to_string(l.bbRotation_[1]) + ", " + std::to_string(l.bbRotation_[2]) + ", " +
               std::to_string(l.bbRotation_[3]) + ")>";
      });

  py::class_<seerep_hdf5_py::BoundingBoxLabel<2>>(m, "BoundingBoxLabel2D")
      .def(py::init<seerep_hdf5_py::InstanceLabel&, std::array<double, 2>&, std::array<double, 2>&,
                    std::array<double, 1>&>(),
           py::arg("label"), py::arg("center"), py::arg("spatial_extent"), py::arg("rotation"))
      .def_readwrite("label", &seerep_hdf5_py::BoundingBoxLabel<2>::label_)
      .def_readwrite("center", &seerep_hdf5_py::BoundingBoxLabel<2>::bbCenter_)
      .def_readwrite("spatial_extent", &seerep_hdf5_py::BoundingBoxLabel<2>::bbSpatialExtent_)
      .def_readwrite("rotation", &seerep_hdf5_py::BoundingBoxLabel<2>::bbRotation_)
      .def("__repr__", [](seerep_hdf5_py::BoundingBoxLabel<2>& l) {
        return "<seerep_hdf5_py.BoundingBoxLabel2D center(" + std::to_string(l.bbCenter_[0]) + ", " +
               std::to_string(l.bbCenter_[1]) + "), extent(" + std::to_string(l.bbSpatialExtent_[0]) + ", " +
               std::to_string(l.bbSpatialExtent_[1]) + "), rotation(" + std::to_string(l.bbRotation_[0]) + ")>";
      });

  py::class_<seerep_hdf5_py::CategorizedBoundingBoxLabel<3>>(m, "CategorizedBoundingBoxLabel3D")
      .def(py::init<const std::string&>(), py::arg("category"))
      .def_readwrite("category", &seerep_hdf5_py::CategorizedBoundingBoxLabel<3>::category_)
      .def_readwrite("labels", &seerep_hdf5_py::CategorizedBoundingBoxLabel<3>::labels_)
      .def("addLabel", &seerep_hdf5_py::CategorizedBoundingBoxLabel<3>::addLabel, py::arg("label"))
      .def("__repr__", [](seerep_hdf5_py::CategorizedBoundingBoxLabel<3>& l) {
        return "<seerep_hdf5_py.CategorizedBoundingBoxLabel3D '" + l.category_ + "'>";
      });

  py::class_<seerep_hdf5_py::CategorizedBoundingBoxLabel<2>>(m, "CategorizedBoundingBoxLabel2D")
      .def(py::init<const std::string&>(), py::arg("category"))
      .def_readwrite("category", &seerep_hdf5_py::CategorizedBoundingBoxLabel<2>::category_)
      .def_readwrite("labels", &seerep_hdf5_py::CategorizedBoundingBoxLabel<2>::labels_)
      .def("addLabel", &seerep_hdf5_py::CategorizedBoundingBoxLabel<2>::addLabel, py::arg("label"))
      .def("__repr__", [](seerep_hdf5_py::CategorizedBoundingBoxLabel<2>& l) {
        return "<seerep_hdf5_py.CategorizedBoundingBoxLabel2D '" + l.category_ + "'>";
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
        return "<seerep_hdf5_py.TfTransform '" + l.frameId_ + "' '" + l.childFrameId_ + "'>";
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
           [](seerep_hdf5_py::Hdf5FileWrapper& f) { return "<seerep_hdf5_py.File '" + f.getFile()->getName() + "'>"; });

  py::class_<seerep_hdf5_py::Hdf5PyImage>(m, "ImageIO")
      .def(py::init<seerep_hdf5_py::Hdf5FileWrapper&>(), py::arg("hdf5_file"))
      .def("getImages", &seerep_hdf5_py::Hdf5PyImage::getImages)
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
