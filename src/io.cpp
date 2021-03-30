#include "ag_proto_hdf5/io.h"

#include <highfive/H5DataSet.hpp>

namespace ag_proto_hdf5{

AGProtoHDF5IO::AGProtoHDF5IO(HighFive::File& file)
    : file(file){}


void AGProtoHDF5IO::writeHeaderAttributes(HighFive::DataSet& data_set, const ag::Header& header)
{
  data_set.createAttribute(HEADER_STAMP_SECONDS, header.stamp().seconds());
  data_set.createAttribute(HEADER_STAMP_NANOS, header.stamp().nanos());
  data_set.createAttribute(HEADER_FRAME_ID, header.frame_id());
  data_set.createAttribute(HEADER_SEQ, header.seq());
}

ag::Header AGProtoHDF5IO::readHeaderAttributes(HighFive::DataSet& data_set)
{
  ag::Header header;

  int64_t seconds;
  int32_t nanos;
  uint32_t seq;

  data_set.getAttribute(HEADER_FRAME_ID).read(header.mutable_frame_id());

  data_set.getAttribute(HEADER_STAMP_SECONDS).read(seconds);
  data_set.getAttribute(HEADER_STAMP_NANOS).read(nanos);
  data_set.getAttribute(HEADER_SEQ).read(seq);

  header.set_seq(seq);
  header.mutable_stamp()->set_seconds(seconds);
  header.mutable_stamp()->set_nanos(nanos);

  return header;
}

void AGProtoHDF5IO::writeImage(const std::string& id, const ag::Image& image)
{
  if(!file.exist(id))
  {
    HighFive::DataSpace data_space(image.row_step(), image.height());
    HighFive::DataSet data_set = file.createDataSet<unsigned char>(id, data_space);
    data_set.write(image.data().c_str());
    data_set.createAttribute(IMAGE_HEIGHT, image.height());
    data_set.createAttribute(IMAGE_WIDTH, image.width());
    data_set.createAttribute(IMAGE_ENCODING, image.encoding());
    data_set.createAttribute(IMAGE_IS_BIGENDIAN, image.is_bigendian());
    data_set.createAttribute(IMAGE_STEP, image.step());
    writeHeaderAttributes(data_set, image.header());
  }
}


std::optional<ag::Image> AGProtoHDF5IO::readImage(const std::string& id)
{
  if(!file.exist(id)) return std::nullopt;

  HighFive::DataSet data_set = file.getDataSet(id);

  ag::Image image;
  data_set.read(image.mutable_data());
  *image.mutable_header() = readHeaderAttributes(data_set, HighFive::DataType());
  return image;
}

} /* namespace ag_proto_hdf5 */
