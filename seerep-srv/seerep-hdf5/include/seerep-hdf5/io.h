#ifndef SEEREP_HDF5__IO_H_
#define SEEREP_HDF5__IO_H_

// highfive
#include <highfive/H5File.hpp>

// seerep-msgs
#include <seerep-msgs/image.pb.h>
#include <seerep-msgs/point_cloud_2.pb.h>
#include <seerep-msgs/point.pb.h>
#include <seerep-msgs/quaternion.pb.h>
#include <seerep-msgs/pose.pb.h>
#include <seerep-msgs/pose_stamped.pb.h>

// std
#include <optional>

namespace seerep_hdf5
{
class SeerepHDF5IO
{
public:
  SeerepHDF5IO(HighFive::File& file);

  void writeHeaderAttributes(HighFive::DataSet& data_set, const seerep::Header& header);

  seerep::Header readHeaderAttributes(HighFive::DataSet& data_set);

  void writeImage(const std::string& id, const seerep::Image& image);

  std::optional<seerep::Image> readImage(const std::string& id);

  void writePointFieldAttributes(HighFive::DataSet& data_set,
                                 const google::protobuf::RepeatedPtrField<seerep::PointField> repeatedPointField);

  google::protobuf::RepeatedPtrField<seerep::PointField> readPointFieldAttributes(HighFive::DataSet& data_set);

  void writePointCloud2(const std::string& id, const seerep::PointCloud2& pointcloud2);

  std::optional<seerep::PointCloud2> readPointCloud2(const std::string& id);

  void writePointAttributes(HighFive::DataSet& data_set, const seerep::Point& point, const std::string& prefix = "");

  void writePoint(const std::string& id, const seerep::Point& point);

  void writeQuaternionAttributes(HighFive::DataSet& data_set, const seerep::Quaternion& quaternion,
                                 const std::string& prefix = "");

  void writeQuaternion(const std::string& id, const seerep::Quaternion& quaternion);

  void writePose(const std::string& id, const seerep::Pose& pose);

  void writePoseStamped(const std::string& id, const seerep::PoseStamped& pose);

private:
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

  HighFive::File file;
  std::mutex write_mtx;
};

} /* namespace seerep_hdf5 */

#endif /* SEEREP_HDF5__IO_H_ */
