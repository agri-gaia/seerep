#ifndef SEEREP_HDF5_PB_HDF5_PB_GENERAL_H_
#define SEEREP_HDF5_PB_HDF5_PB_GENERAL_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include <seerep-hdf5-core/hdf5-core-general.h>

// seerep-msgs
#include <seerep-msgs/boundingbox2d_labeled.pb.h>
#include <seerep-msgs/boundingbox_labeled.pb.h>

// std
#include <boost/geometry.hpp>
#include <filesystem>
#include <optional>

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_hdf5_pb
{
class Hdf5PbGeneral
{
protected:
  Hdf5PbGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  std::optional<std::string> readFrameId(const std::string& datatypeGroup, const std::string& uuid);

  std::vector<std::string> getGroupDatasets(const std::string& id);

  void checkExists(const std::string& id);

  //################
  // Attributes
  //################
  template <typename T>
  void writeAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField, T value);

  template <typename T>
  T getAttribute(const std::string& id, const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField);

  void deleteAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField);

  template <class T>
  void writeHeaderAttributes(HighFive::AnnotateTraits<T>& object, const seerep::Header& header);

  template <class T>
  seerep::Header readHeaderAttributes(HighFive::AnnotateTraits<T>& object, const std::string& id);

  //################
  // AABB
  //################
  void writeAABB(
      const std::string& datatypeGroup, const std::string& uuid,
      const boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb);

  void
  readAABB(const std::string& datatypeGroup, const std::string& uuid,
           boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb);

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
  void
  writeBoundingBoxLabeled(const std::string& datatypeGroup, const std::string& uuid,
                          const google::protobuf::RepeatedPtrField<::seerep::BoundingBoxLabeled>& boundingboxLabeled);

  void writeBoundingBox2DLabeled(
      const std::string& datatypeGroup, const std::string& uuid,
      const google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeled>& boundingbox2DLabeled);

  std::optional<google::protobuf::RepeatedPtrField<::seerep::BoundingBox2DLabeled>>
  readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid);

  //################
  // Labels General
  //################
  void
  writeLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid,
                     const google::protobuf::RepeatedPtrField<seerep::LabelWithInstance>& labelsGeneralWithInstances);

  std::optional<google::protobuf::RepeatedPtrField<seerep::LabelWithInstance>>
  readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid);

  //################
  // Project
  //################
  void writeProjectname(const std::string& projectname);

  std::string readProjectname();

  void writeProjectFrameId(const std::string& frameId);

  std::string readProjectFrameId();

protected:
  std::shared_ptr<HighFive::File> m_file;
  std::shared_ptr<std::mutex> m_write_mtx;
};

}  // namespace seerep_hdf5_pb

#include "impl/hdf5-pb-general.hpp"  // NOLINT

#endif /* SEEREP_HDF5_PB_HDF5_PB_GENERAL_H_ */
