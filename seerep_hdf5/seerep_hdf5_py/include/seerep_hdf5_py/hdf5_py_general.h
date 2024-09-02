#ifndef SEEREP_HDF5_PY_HDF5_PY_GENERAL_H_
#define SEEREP_HDF5_PY_HDF5_PY_GENERAL_H_

// highfive
#include <highfive/H5File.hpp>

// seerep_hdf5
#include <seerep_hdf5_core/hdf5_core_general.h>

// std
#include <boost/geometry.hpp>
#include <filesystem>
#include <optional>

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_hdf5_py
{
class Hdf5FileWrapper
{
public:
  Hdf5FileWrapper(const std::string& filename)
    : filePtr_(std::make_shared<HighFive::File>(
          filename, HighFive::File::ReadWrite | HighFive::File::Create))
    , writeMutex_(std::make_shared<std::mutex>())
  {
  }

  std::shared_ptr<HighFive::File>& getFile()
  {
    return filePtr_;
  }

  std::shared_ptr<std::mutex>& getMutex()
  {
    return writeMutex_;
  }

  void createProject(const std::string& projectName,
                     const std::string& rootFrameId);

  void setProjectGeolocation(const std::string& crsString,
                             const std::string& ellipsoid, double latitude,
                             double longitude, double altitude);

private:
  std::shared_ptr<HighFive::File> filePtr_;
  std::shared_ptr<std::mutex> writeMutex_;
};

struct InstanceLabel
{
public:
  InstanceLabel(const std::string& label, float confidence,
                const std::string& instanceUuid)
    : label_(label), confidence_(confidence), instanceUuid_(instanceUuid)
  {
  }

  std::string label_ = "";
  float confidence_ = 0.0;
  std::string instanceUuid_ = "";
};

struct GeneralLabel
{
public:
  GeneralLabel(const std::string& category) : category_(category)
  {
  }

  void addLabel(InstanceLabel& label)
  {
    labels_.push_back(label);
  }

  std::string category_ = "";
  std::vector<InstanceLabel> labels_;
};

template <int NumDimensions>
struct BoundingBoxLabel
{
public:
  using RotationType = std::array < double, NumDimensions<3 ? 1 : 4>;

  BoundingBoxLabel(InstanceLabel& label,
                   std::array<double, NumDimensions>& bbCenter,
                   std::array<double, NumDimensions>& bbSpatialExtent,
                   RotationType& bbRotation)
    : label_(label)
    , bbCenter_(bbCenter)
    , bbSpatialExtent_(bbSpatialExtent)
    , bbRotation_(bbRotation)
  {
  }

  InstanceLabel label_;
  std::array<double, NumDimensions> bbCenter_;
  std::array<double, NumDimensions> bbSpatialExtent_;
  RotationType bbRotation_;
};

template <int NumDimensions>
struct CategorizedBoundingBoxLabel
{
public:
  CategorizedBoundingBoxLabel(const std::string& category) : category_(category)
  {
  }

  void addLabel(BoundingBoxLabel<NumDimensions>& label)
  {
    labels_.push_back(label);
  }

  std::string category_ = "";
  std::vector<BoundingBoxLabel<NumDimensions>> labels_;
};

class Hdf5PyGeneral : public virtual seerep_hdf5_core::Hdf5CoreGeneral
{
public:
  Hdf5PyGeneral(Hdf5FileWrapper& hdf5File);

  // #########
  //  Project
  // #########

  void setProjectGeolocation(const std::string& crsString,
                             const std::string& ellipsoid, double latitude,
                             double longitude, double altitude);

protected:
  // ################
  //  BoundingBoxes
  // ################

  template <int NumDimensions>
  void writeBoundingBoxLabeled(
      const std::string& dataGroupId,
      const std::vector<
          seerep_hdf5_py::CategorizedBoundingBoxLabel<NumDimensions>>& bbLabels);

  template <int NumDimensions>
  std::vector<seerep_hdf5_py::CategorizedBoundingBoxLabel<NumDimensions>>
  readBoundingBoxLabeled(const std::string& dataGroupId);

  // ################
  //  Labels General
  // ################

  void writeLabelsGeneral(
      const std::string& dataGroupId,
      const std::vector<seerep_hdf5_py::GeneralLabel>& generalLabels);

  std::vector<seerep_hdf5_py::GeneralLabel>
  readLabelsGeneral(const std::string& dataGroupId);
};

}  // namespace seerep_hdf5_py

#include "impl/hdf5_py_general.hpp"  // NOLINT

#endif /* SEEREP_HDF5_PY_HDF5_PY_GENERAL_H_ */
