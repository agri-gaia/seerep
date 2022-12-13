#ifndef SEEREP_HDF5_FB_GENERAL_H_
#define SEEREP_HDF5_FB_GENERAL_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-hdf5
#include <seerep-hdf5-core/hdf5-core-general.h>

// seerep-msgs
#include <seerep-msgs/boundingbox2d_labeled_generated.h>
#include <seerep-msgs/boundingbox_labeled_generated.h>
#include <seerep-msgs/header_generated.h>
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
// make nested flatbuffers readable
typedef flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeled>> BoundingBoxesLabeledFb;
typedef flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeled>> BoundingBoxes2dLabeledFb;
typedef flatbuffers::Vector<flatbuffers::Offset<seerep::fb::LabelWithInstance>> GeneralLabelsFb;
typedef flatbuffers::Vector<flatbuffers::Offset<seerep::fb::UnionMapEntry>> AttributeMapsFb;

class Hdf5FbGeneral : public virtual seerep_hdf5_core::Hdf5CoreGeneral
{
protected:
  Hdf5FbGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx);

  // Attributes can be written to DataSets or DataGroups
  void writeAttributeMap(const std::shared_ptr<HighFive::DataSet> dataSetPtr, const AttributeMapsFb* attributes);

  template <class T>
  flatbuffers::Offset<AttributeMapsFb> readAttributeMap(HighFive::AnnotateTraits<T>& object,
                                                        flatbuffers::grpc::MessageBuilder& builder);

  template <class T>
  void writeHeaderAttributes(HighFive::AnnotateTraits<T>& object, const seerep::fb::Header* header);

  template <class T>
  flatbuffers::Offset<seerep::fb::Header> readHeaderAttributes(flatbuffers::grpc::MessageBuilder& builder,
                                                               HighFive::AnnotateTraits<T>& object,
                                                               std::string uuidMsg);

  // Bounding boxes are stored as a DataSet
  void writeBoundingBoxLabeled(const std::string& datatypeGroup, const std::string& uuid,
                               const BoundingBoxesLabeledFb* boundingboxLabeled);

  void writeBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid,
                                 const BoundingBoxes2dLabeledFb* boundingbox2dLabeled);

  flatbuffers::Offset<BoundingBoxes2dLabeledFb> readBoundingBoxes2DLabeled(const std::string& datatypeGroup,
                                                                           const std::string& id,
                                                                           flatbuffers::grpc::MessageBuilder& builder);
  // General Labels are stored as a DataSet
  void writeLabelsGeneral(const std::string& datatypeGroup, const std::string& sid,
                          const GeneralLabelsFb* labelsGeneral);

  flatbuffers::Offset<GeneralLabelsFb> readGeneralLabels(const std::string& datatypeGroup, const std::string& id,
                                                         flatbuffers::grpc::MessageBuilder& builder);
};

}  // namespace seerep_hdf5_fb

#include "impl/hdf5-fb-general.hpp"  // NOLINT

#endif /* SEEREP_HDF5_FB_GENERAL_H_ */
