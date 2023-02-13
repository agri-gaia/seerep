#ifndef SEEREP_HDF5_PY_HDF5_PY_GENERAL_H_
#define SEEREP_HDF5_PY_HDF5_PY_GENERAL_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include <seerep-hdf5-core/hdf5-core-general.h>

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
    : filePtr_(std::make_shared<HighFive::File>(filename, HighFive::File::ReadWrite | HighFive::File::Create))
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

  void createProject(const std::string& projectName, const std::string& rootFrameId);

  void setProjectGeolocation(const std::string& coordinateSystem, const std::string& ellipsoid, double latitude,
                             double longitude, double altitude);

private:
  std::shared_ptr<HighFive::File> filePtr_;
  std::shared_ptr<std::mutex> writeMutex_;
};

struct InstanceLabel
{
public:
  InstanceLabel(const std::string& label, const std::string& instanceUuid) : label_(label), instanceUuid_(instanceUuid)
  {
  }

  std::string label_ = "";
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
  BoundingBoxLabel(InstanceLabel& label, std::array<double, NumDimensions>& minPoint,
                   std::array<double, NumDimensions>& maxPoint)
    : label_(label), minPoint_(minPoint), maxPoint_(maxPoint)
  {
  }

  InstanceLabel label_;
  std::array<double, NumDimensions> minPoint_;
  std::array<double, NumDimensions> maxPoint_;
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

  void setProjectGeolocation(const std::string& coordinateSystem, const std::string& ellipsoid, double latitude,
                             double longitude, double altitude);

protected:
  // ################
  //  BoundingBoxes
  // ################

  template <int NumDimensions>
  void writeBoundingBoxLabeled(const std::string& dataGroupId,
                               const std::vector<seerep_hdf5_py::CategorizedBoundingBoxLabel<NumDimensions>>& bbLabels);

  template <int NumDimensions>
  std::vector<seerep_hdf5_py::CategorizedBoundingBoxLabel<NumDimensions>>
  readBoundingBoxLabeled(const std::string& dataGroupId);

  // ################
  //  Labels General
  // ################

  void writeLabelsGeneral(const std::string& dataGroupId,
                          const std::vector<seerep_hdf5_py::GeneralLabel>& generalLabels);

  std::vector<seerep_hdf5_py::GeneralLabel> readLabelsGeneral(const std::string& dataGroupId);
};

}  // namespace seerep_hdf5_py

#include "impl/hdf5-py-general.hpp"  // NOLINT

#endif /* SEEREP_HDF5_PY_HDF5_PY_GENERAL_H_ */
