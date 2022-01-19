#ifndef SEEREP_HDF5__IO_H_
#define SEEREP_HDF5__IO_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-msgs
#include <seerep-msgs/boundingbox_labeled.pb.h>
#include <seerep-msgs/image.pb.h>
#include <seerep-msgs/point.pb.h>
#include <seerep-msgs/point_cloud_2.pb.h>
#include <seerep-msgs/pose.pb.h>
#include <seerep-msgs/pose_stamped.pb.h>
#include <seerep-msgs/quaternion.pb.h>
#include <seerep-msgs/transform_stamped.pb.h>

// std
#include <optional>

#include <boost/geometry.hpp>

namespace seerep_hdf5
{
class SeerepHDF5IO
{
public:
  template <typename T>
  void writeAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField, T value);

  template <typename T>
  T getAttribute(const std::string& id, const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField);

  void deleteAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField);

  SeerepHDF5IO(HighFive::File& file);

  void writeImage(const std::string& id, const seerep::Image& image);

  // void writeImageLabeled(const std::string& id, const seerep::ImageLabeled& imageLabeled);

  std::optional<seerep::Image> readImage(const std::string& id);

  void writePointCloud2(const std::string& uuid, const seerep::PointCloud2& pointcloud2);

  // void writePointCloud2Labeled(const std::string& id, const seerep::PointCloud2Labeled& pointcloud2Labeled);

  void
  writeBoundingBoxLabeled(const std::string& datatypeGroup, const std::string& uuid,
                          const google::protobuf::RepeatedPtrField<::seerep::BoundingBoxLabeled>& boundingboxLabeled);

  void writeBoundingBox2DLabeled(
      const std::string& datatypeGroup, const std::string& uuid,
      const google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeled>& boundingbox2DLabeled);

  std::optional<google::protobuf::RepeatedPtrField<::seerep::BoundingBox2DLabeled>>
  readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid);

  void writeLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid,
                          const google::protobuf::RepeatedPtrField<std::string>& labelsGeneral);

  std::optional<google::protobuf::RepeatedPtrField<std::string>> readLabelsGeneral(const std::string& datatypeGroup,
                                                                                   const std::string& uuid);

  void writeAABB(
      const std::string& datatypeGroup, const std::string& uuid,
      const boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb);

  void
  readAABB(const std::string& datatypeGroup, const std::string& uuid,
           boost::geometry::model::box<boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian>>& aabb);

  bool hasAABB(const std::string& datatypeGroup, const std::string& uuid);

  int64_t readTimeFromRaw(const std::string& datatypeGroup, const std::string& uuid);
  int64_t readTime(const std::string& datatypeGroup, const std::string& uuid);

  void writeTimeToRaw(const std::string& datatypeGroup, const std::string& uuid, const int64_t& time);
  void writeTime(const std::string& datatypeGroup, const std::string& uuid, const int64_t& time);

  bool hasTimeRaw(const std::string& datatypeGroup, const std::string& uuid);
  bool hasTime(const std::string& datatypeGroup, const std::string& uuid);

  std::optional<seerep::PointCloud2> readPointCloud2(const std::string& id);

  void writePoint(const std::string& id, const seerep::Point& point);

  void writeQuaternion(const std::string& id, const seerep::Quaternion& quaternion);

  void writePose(const std::string& id, const seerep::Pose& pose);

  void writePoseStamped(const std::string& id, const seerep::PoseStamped& pose);

  std::vector<std::string> getGroupDatasets(const std::string& id);

  void writeProjectname(const std::string& projectname);

  std::string readProjectname();

  void writeTransformStamped(const seerep::TransformStamped& tf);

  // std::optional<seerep::TransformStamped> readTransformStamped(const std::string& id);

private:
  void writeHeaderAttributes(HighFive::DataSet& data_set, const seerep::Header& header);

  seerep::Header readHeaderAttributes(HighFive::DataSet& data_set);

  void writePointFieldAttributes(HighFive::DataSet& data_set,
                                 const google::protobuf::RepeatedPtrField<seerep::PointField> repeatedPointField);

  google::protobuf::RepeatedPtrField<seerep::PointField> readPointFieldAttributes(HighFive::DataSet& data_set);

  void writeQuaternionAttributes(HighFive::DataSet& data_set, const seerep::Quaternion& quaternion,
                                 const std::string& prefix = "");

  void writePointAttributes(HighFive::DataSet& data_set, const seerep::Point& point, const std::string& prefix = "");

  const std::string SIZE = "size";
  const std::string CLASS = "CLASS";
  // image / pointcloud attribute keys
  const std::string HEIGHT = "height";
  const std::string WIDTH = "width";
  const std::string ENCODING = "encoding";
  const std::string IS_BIGENDIAN = "is_bigendian";
  const std::string ROW_STEP = "row_step";
  const std::string POINT_STEP = "point_step";
  const std::string IS_DENSE = "is_dense";

  // pointcloud fields attribute keys
  const std::string FIELD_NAME = "field_name_";
  const std::string FIELD_OFFSET = "field_offset_";
  const std::string FIELD_DATATYPE = "field_datatype_";
  const std::string FIELD_COUNT = "field_count_";

  // header attribute keys
  const std::string HEADER_STAMP_SECONDS = "header_stamp_seconds";
  const std::string HEADER_STAMP_NANOS = "header_stamp_nanos";
  const std::string HEADER_FRAME_ID = "header_frame_id";
  const std::string HEADER_SEQ = "header_seq";

  // point and quaternion attribute keys
  const std::string X = "x";
  const std::string Y = "y";
  const std::string Z = "z";
  const std::string W = "w";

  // pose attribute keys
  const std::string POSE = "pose";
  const std::string POSITION = "position";
  const std::string ORIENTATION = "orientation";

  const std::string AABB_FIELD = "AABB";

  const std::string PROJECTNAME = "projectname";

  // dataset names
  const std::string RAWDATA = "rawdata";
  const std::string LABELGENERAL = "labelGeneral";
  const std::string LABELBB = "labelBB";
  const std::string LABELBBBOXES = "labelBBBoxes";

  HighFive::File m_file;
  std::mutex m_write_mtx;

public:
  // datatype group names in hdf5
  inline static const std::string HDF5_GROUP_IMAGE = "images";
  inline static const std::string HDF5_GROUP_POINTCLOUD = "pointclouds";
  inline static const std::string HDF5_GROUP_TF = "tf";
};

} /* namespace seerep_hdf5 */

#endif /* SEEREP_HDF5__IO_H_ */
