#ifndef SEEREP_HDF5_FB_GENERAL_H_
#define SEEREP_HDF5_FB_GENERAL_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include <seerep-hdf5-core/hdf5-core-general.h>

// seerep-msgs
#include <seerep-msgs/boundingbox2d_labeled_generated.h>
#include <seerep-msgs/boundingbox_labeled_generated.h>
#include <seerep-msgs/union_map_entry_generated.h>

// grpc / flatbuffer
#include <grpcpp/grpcpp.h>

#include "flatbuffers/grpc.h"

// std
#include <boost/geometry.hpp>
#include <filesystem>
#include <mutex>
#include <optional>

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
namespace seerep_hdf5_fb
{
class Hdf5FbGeneral
{
protected:
  Hdf5FbGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  std::optional<std::string> readFrameId(const std::string& datatypeGroup, const std::string& uuid);

  std::vector<std::string> getGroupDatasets(const std::string& id);

  void checkExists(const std::string& id);

  //################
  // Attributes
  //################
  void writeAttributeMap(const std::shared_ptr<HighFive::DataSet> dataSetPtr,
                         const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::UnionMapEntry>>& attributes);
  template <class T>
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<seerep::fb::UnionMapEntry>>>
  readAttributeMap(HighFive::AnnotateTraits<T>& object, flatbuffers::grpc::MessageBuilder& builder);

  template <typename T>
  void writeAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField, T value);

  template <typename T>
  T getAttribute(const std::string& id, const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField);

  void deleteAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField);

  template <class T>
  void writeHeaderAttributes(HighFive::AnnotateTraits<T>& object, const seerep::fb::Header& header);

  template <class T>
  flatbuffers::Offset<seerep::fb::Header> readHeaderAttributes(HighFive::AnnotateTraits<T>& object, std::string uuidMsg,
                                                               flatbuffers::grpc::MessageBuilder& builder);

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
  template <class T>
  void readTimeFromAnnotateTraits(const std::string& id, int64_t& value,
                                  const HighFive::AnnotateTraits<T>& highFiveObject, const std::string& attribute);

  void writeTimeToRaw(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
                      const int64_t& nanos);
  void writeTime(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs, const int64_t& nanos);
  template <class T>
  void writeTimeToAnnotateTraits(const int64_t& value, HighFive::AnnotateTraits<T>& highFiveObject,
                                 const std::string& attribute);

  bool hasTimeRaw(const std::string& datatypeGroup, const std::string& uuid);
  bool hasTime(const std::string& datatypeGroup, const std::string& uuid);

  //################
  // BoundingBoxes
  //################
  void writeBoundingBoxLabeled(
      const std::string& datatypeGroup, const std::string& uuid,
      const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeled>>* boundingboxLabeled);

  void writeBoundingBox2DLabeled(
      const std::string& datatypeGroup, const std::string& uuid,
      const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeled>>* boundingbox2DLabeled);

  void readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid,
                                std::vector<std::string>& labels, std::vector<std::vector<double>>& boundingBoxes,
                                std::vector<std::string>& instances);

  //################
  // Labels General
  //################
  void writeLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid,
                          const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::LabelWithInstance>>* labelsGeneral);

  void readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid, std::vector<std::string>& labels,
                         std::vector<std::string>& instances);

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
  boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

}  // namespace seerep_hdf5_fb

#include "impl/hdf5-fb-general.hpp"  // NOLINT

#endif /* SEEREP_HDF5_FB_GENERAL_H_ */
