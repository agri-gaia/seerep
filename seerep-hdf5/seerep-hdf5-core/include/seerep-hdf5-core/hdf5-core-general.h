#ifndef SEEREP_HDF5_HDF5_GENERAL_CORE_H_
#define SEEREP_HDF5_HDF5_GENERAL_CORE_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-msgs
#include <seerep-msgs/aabb.h>
#include <seerep-msgs/dataset-indexable.h>

// std
#include <boost/geometry.hpp>
#include <mutex>
#include <optional>

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_hdf5_core
{
class Hdf5CoreGeneral
{
public:
  Hdf5CoreGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  std::vector<std::string> getGroupDatasets(const std::string& id);

  //################
  // Attributes
  //################
  template <typename T, class C>
  T getAttribute(const std::string& id, const HighFive::AnnotateTraits<C>& object, std::string attributeField);

  // //################
  // // Project
  // //################
  void writeProjectname(const std::string& projectname);
  std::string readProjectname();

  void writeProjectFrameId(const std::string& frameId);
  std::string readProjectFrameId();

public:
  // header attribute keys
  inline static const std::string HEADER_STAMP_SECONDS = "header_stamp_seconds";
  inline static const std::string HEADER_STAMP_NANOS = "header_stamp_nanos";
  inline static const std::string HEADER_FRAME_ID = "header_frame_id";
  inline static const std::string HEADER_SEQ = "header_seq";

  inline static const std::string AABB_FIELD = "AABB";

  inline static const std::string PROJECTNAME = "projectname";
  inline static const std::string PROJECTFRAMEID = "projectframeid";

  // dataset names
  inline static const std::string RAWDATA = "rawdata";
  inline static const std::string LABELGENERAL = "labelGeneral";
  inline static const std::string LABELGENERALINSTANCES = "labelGeneralInstances";
  inline static const std::string LABELBB = "labelBB";
  inline static const std::string LABELBBBOXES = "labelBBBoxes";
  inline static const std::string LABELBBINSTANCES = "labelBBInstances";

protected:
  std::shared_ptr<HighFive::File> m_file;
  std::shared_ptr<std::mutex> m_write_mtx;
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

}  // namespace seerep_hdf5_core

#include "impl/hdf5-core-general.hpp"  // NOLINT

#endif /* SEEREP_HDF5_HDF5_GENERAL_CORE_H_ */
