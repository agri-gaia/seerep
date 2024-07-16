#ifndef SEEREP_HDF5_PB_HDF5_PB_GENERAL_H_
#define SEEREP_HDF5_PB_HDF5_PB_GENERAL_H_

// highfive
#include <highfive/H5File.hpp>

// seerep_hdf5
#include <seerep_hdf5_core/hdf5_core_general.h>

// seerep-msgs
#include <seerep_msgs/header.pb.h>
#include <seerep_msgs/label_category.pb.h>

// std
#include <boost/geometry.hpp>
#include <filesystem>
#include <optional>

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_hdf5_pb
{
class Hdf5PbGeneral : public virtual seerep_hdf5_core::Hdf5CoreGeneral
{
protected:
  Hdf5PbGeneral(std::shared_ptr<HighFive::File>& file,
                std::shared_ptr<std::mutex>& write_mtx);

  // ################
  //  Attributes
  // ################
  template <class T>
  void writeHeaderAttributes(HighFive::AnnotateTraits<T>& object,
                             const seerep::pb::Header& header);

  template <class T>
  seerep::pb::Header readHeaderAttributes(HighFive::AnnotateTraits<T>& object,
                                          const std::string& id);

  // ################
  //  Labels
  // ################
  void writeLabels(
      const std::string& datatypeGroup, const std::string& uuid,
      const google::protobuf::RepeatedPtrField<seerep::pb::LabelCategory>&
          labels);

  std::optional<google::protobuf::RepeatedPtrField<seerep::pb::LabelCategory>>
  readLabels(const std::string& datatypeGroup, const std::string& uuid);
};

}  // namespace seerep_hdf5_pb

#include "impl/hdf5_pb_general.hpp"  // NOLINT

#endif /* SEEREP_HDF5_PB_HDF5_PB_GENERAL_H_ */
