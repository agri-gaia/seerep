#ifndef SEEREP_HDF5_IO_GENERAL_CORE_H_
#define SEEREP_HDF5_IO_GENERAL_CORE_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-msgs
#include <seerep-msgs/dataset-indexable.h>
#include <seerep-msgs/aabb.h>

// std
#include <optional>
#include <mutex>

#include <boost/geometry.hpp>

namespace seerep_io_core
{
class IoCoreGeneral
{
public:
  IoCoreGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  std::optional<std::string> readFrameId(const std::string& datatypeGroup, const std::string& uuid);

  std::vector<std::string> getGroupDatasets(const std::string& id);

  //################
  // Attributes
  //################
  template <typename T>
  void writeAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField, T value);

  template <typename T>
  T getAttribute(const std::string& id, const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField);

  void deleteAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField);

  std::optional<seerep_core_msgs::DatasetIndexable> readDataForIndices(const std::string& datatypeGroup,
                                                                       const std::string& uuid);

  //################
  // AABB
  //################
  void writeAABB(const std::string& datatypeGroup, const std::string& uuid, const seerep_core_msgs::AABB& aabb);

  void readAABB(const std::string& datatypeGroup, const std::string& uuid, seerep_core_msgs::AABB& aabb);

  bool hasAABB(const std::string& datatypeGroup, const std::string& uuid);

  //################
  // Time
  //################
  void readTimeFromRaw(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs, int64_t& nanos);
  void readTime(const std::string& datatypeGroup, const std::string& uuid, int64_t& secs, int64_t& nanos);

  void writeTimeToRaw(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
                      const int64_t& nanos);
  void writeTime(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs, const int64_t& nanos);

  bool hasTimeRaw(const std::string& datatypeGroup, const std::string& uuid);
  bool hasTime(const std::string& datatypeGroup, const std::string& uuid);

  //################
  // BoundingBoxes
  //################
  void readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid,
                                std::vector<std::string> labels, std::vector<std::vector<double>> boundingBoxes);

  //################
  // Labels General
  //################
  void readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid, std::vector<std::string> labels);

  //################
  // Project
  //################
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
  inline static const std::string LABELBB = "labelBB";
  inline static const std::string LABELBBBOXES = "labelBBBoxes";

protected:
  std::shared_ptr<HighFive::File> m_file;
  std::shared_ptr<std::mutex> m_write_mtx;
};

}  // namespace seerep_io_core

#include "impl/io-core-general.hpp"  // NOLINT

#endif /* SEEREP_HDF5_IO_GENERAL_CORE_H_ */
