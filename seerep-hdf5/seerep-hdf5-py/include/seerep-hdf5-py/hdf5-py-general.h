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
    : file_ptr_(std::make_shared<HighFive::File>(filename, HighFive::File::ReadWrite | HighFive::File::Create))
    , write_mutex_(std::make_shared<std::mutex>())
  {
  }

  std::shared_ptr<HighFive::File>& getFile()
  {
    return file_ptr_;
  }

  std::shared_ptr<std::mutex>& getMutex()
  {
    return write_mutex_;
  }

  void createProject(const std::string& project_name, const std::string& root_frame_id);

  void setProjectGeolocation(const std::string& coordinate_system, const std::string& ellipsoid, double latitude,
                             double longitude, double altitude);

private:
  std::shared_ptr<HighFive::File> file_ptr_;
  std::shared_ptr<std::mutex> write_mutex_;
};

struct InstanceLabel
{
public:
  InstanceLabel(const std::string& new_label, const std::string& new_instance_uuid)
    : label(new_label), instance_uuid(new_instance_uuid)
  {
  }

  std::string label = "";
  std::string instance_uuid = "";
};

struct GeneralLabel
{
public:
  GeneralLabel(const std::string& new_category) : category(new_category)
  {
  }

  void addLabel(InstanceLabel& label)
  {
    labels.push_back(label);
  }

  std::string category = "";
  std::vector<InstanceLabel> labels;
};

template <int NumDimensions>
struct BoundingBoxLabel
{
public:
  BoundingBoxLabel(InstanceLabel& new_label, std::array<double, NumDimensions>& new_min_point,
                   std::array<double, NumDimensions>& new_max_point)
    : label(new_label), min_point(new_min_point), max_point(new_max_point)
  {
  }

  InstanceLabel label;
  std::array<double, NumDimensions> min_point;
  std::array<double, NumDimensions> max_point;
};

template <int NumDimensions>
struct CategorizedBoundingBoxLabel
{
public:
  CategorizedBoundingBoxLabel(const std::string& new_category) : category(new_category)
  {
  }

  void addLabel(BoundingBoxLabel<NumDimensions>& label)
  {
    labels.push_back(label);
  }

  std::string category = "";
  std::vector<BoundingBoxLabel<NumDimensions>> labels;
};

class Hdf5PyGeneral : public virtual seerep_hdf5_core::Hdf5CoreGeneral
{
public:
  Hdf5PyGeneral(Hdf5FileWrapper& hdf5_file);

  // #########
  //  Project
  // #########

  void setProjectGeolocation(const std::string& coordinate_system, const std::string& ellipsoid, double latitude,
                             double longitude, double altitude);

protected:
  // ################
  //  BoundingBoxes
  // ################

  template <int NumDimensions>
  void writeBoundingBoxLabeled(const std::string& data_group_id,
                               const std::vector<seerep_hdf5_py::CategorizedBoundingBoxLabel<NumDimensions>>& bb_labels);

  template <int NumDimensions>
  std::vector<seerep_hdf5_py::CategorizedBoundingBoxLabel<NumDimensions>>
  readBoundingBoxLabeled(const std::string& data_group_id);

  // ################
  //  Labels General
  // ################

  void writeLabelsGeneral(const std::string& data_group_id,
                          const std::vector<seerep_hdf5_py::GeneralLabel>& general_labels);

  std::vector<seerep_hdf5_py::GeneralLabel> readLabelsGeneral(const std::string& data_group_id);
};

}  // namespace seerep_hdf5_py

#include "impl/hdf5-py-general.hpp"  // NOLINT

#endif /* SEEREP_HDF5_PY_HDF5_PY_GENERAL_H_ */
