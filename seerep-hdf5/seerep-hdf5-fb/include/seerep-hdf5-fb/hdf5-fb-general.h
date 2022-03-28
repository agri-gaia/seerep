#ifndef SEEREP_HDF5_FB_GENERAL_H_
#define SEEREP_HDF5_FB_GENERAL_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-msgs
#include <seerep-msgs/boundingbox2d_labeled_generated.h>
#include <seerep-msgs/boundingbox_labeled_generated.h>

// grpc / flatbuffer
#include <grpcpp/grpcpp.h>

#include "flatbuffers/grpc.h"

// std
#include <boost/geometry.hpp>
#include <mutex>
#include <optional>

namespace seerep_hdf5_fb
{
class Hdf5FbGeneral
{
public:
  Hdf5FbGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

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

  template <class T>
  void writeHeaderAttributes(HighFive::AnnotateTraits<T>& object, const seerep::fb::Header& header);

  template <class T>
  flatbuffers::Offset<seerep::fb::Header> readHeaderAttributes(HighFive::AnnotateTraits<T>& object,
                                                               std::string uuidProject, std::string uuidMsg,
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

  void writeTimeToRaw(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs,
                      const int64_t& nanos);
  void writeTime(const std::string& datatypeGroup, const std::string& uuid, const int64_t& secs, const int64_t& nanos);

  bool hasTimeRaw(const std::string& datatypeGroup, const std::string& uuid);
  bool hasTime(const std::string& datatypeGroup, const std::string& uuid);

  //################
  // BoundingBoxes
  //################
  void writeBoundingBoxLabeled(
      const std::string& datatypeGroup, const std::string& uuid,
      const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeled>>& boundingboxLabeled);

  void writeBoundingBox2DLabeled(
      const std::string& datatypeGroup, const std::string& uuid,
      const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeled>>& boundingbox2DLabeled);

  void readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid,
                                std::vector<std::string> labels, std::vector<std::vector<double>> boundingBoxes);

  //################
  // Labels General
  //################
  void writeLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid,
                          const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>& labelsGeneral);

  void readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid, std::vector<std::string> labels);

  //################
  // Project
  //################
  void writeProjectname(const std::string& projectname);

  std::string readProjectname();

  void writeProjectFrameId(const std::string& frameId);

  std::string readProjectFrameId();

private:
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

}  // namespace seerep_hdf5_fb

#include "impl/hdf5-fb-general.hpp"  // NOLINT

#endif /* SEEREP_HDF5_FB_GENERAL_H_ */
