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

private:
  std::shared_ptr<HighFive::File> file_ptr_;
  std::shared_ptr<std::mutex> write_mutex_;
};

class Hdf5PyGeneral : public virtual seerep_hdf5_core::Hdf5CoreGeneral
{
protected:
  Hdf5PyGeneral(Hdf5FileWrapper& hdf5_file);

  // ################
  //  Attributes
  // ################
  //   template <class T>
  //   void writeHeaderAttributes(HighFive::AnnotateTraits<T>& object, const seerep::Header& header);

  //   template <class T>
  //   seerep::Header readHeaderAttributes(HighFive::AnnotateTraits<T>& object, const std::string& id);

  // ################
  //  BoundingBoxes
  // ################
  //   void writeBoundingBoxLabeled(
  //       const std::string& datatypeGroup, const std::string& uuid,
  //       const google::protobuf::RepeatedPtrField<::seerep::BoundingBoxLabeledWithCategory>& boundingboxLabeled);

  //   void writeBoundingBox2DLabeled(
  //       const std::string& datatypeGroup, const std::string& uuid,
  //       const google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeledWithCategory>& boundingbox2DLabeled);

  //   std::optional<google::protobuf::RepeatedPtrField<::seerep::BoundingBox2DLabeledWithCategory>>
  //   readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid);

  // ################
  //  Labels General
  // ################
  //   void writeLabelsGeneral(
  //       const std::string& datatypeGroup, const std::string& uuid,
  //       const google::protobuf::RepeatedPtrField<seerep::LabelsWithInstanceWithCategory>& labelsGeneralWithInstances);

  //   std::optional<google::protobuf::RepeatedPtrField<seerep::LabelsWithInstanceWithCategory>>
  //   readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid);
};

}  // namespace seerep_hdf5_py

#include "impl/hdf5-py-general.hpp"  // NOLINT

#endif /* SEEREP_HDF5_PY_HDF5_PY_GENERAL_H_ */
