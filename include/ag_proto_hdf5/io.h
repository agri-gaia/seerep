#ifndef AG_PROTO_HDF5__IO_H_
#define AG_PROTO_HDF5__IO_H_

// highfive
#include <highfive/H5File.hpp>

// ag_proto_msgs
#include <ag_proto_msgs/image.pb.h>

// std
#include <optional>


namespace ag_proto_hdf5
{

class AGProtoHDF5IO
{
public:

  AGProtoHDF5IO(HighFive::File& file);

  void writeHeaderAttributes(HighFive::DataSet& data_set, const ag::Header& header);

  ag::Header readHeaderAttributes(HighFive::DataSet& data_set);

  void writeImage(const std::string& id, const ag::Image& image);

  std::optional<ag::Image> readImage(const std::string& id);

private:

  // image attribute keys
  const std::string IMAGE_HEIGHT = "height";
  const std::string IMAGE_WIDTH = "width";
  const std::string IMAGE_ENCODING ="encoding";
  const std::string IMAGE_IS_BIGENDIAN = "is_bigendian";
  const std::string IMAGE_STEP = "step";

  // header attribute keys
  const std::string HEADER_STAMP_SECONDS = "header_stamp_seconds";
  const std::string HEADER_STAMP_NANOS = "header_stamp_nanos";
  const std::string HEADER_FRAME_ID = "header_frame_id";
  const std::string HEADER_SEQ = "header_seq";

  HighFive::File file;

};


} /* namespace ag_proto_hdf5 */

#endif /* AG_PROTO_HDF5__IO_H_ */
