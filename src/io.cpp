#include "ag_proto_hdf5/io.h"

#include <highfive/H5DataSet.hpp>

namespace ag_proto_hdf5{

AGProtoHDF5IO::AGProtoHDF5IO(HighFive::File& file)
    : file(file){}


void AGProtoHDF5IO::writeHeaderAttributes(HighFive::DataSet& data_set, const ag::Header& header)
{
  if(!data_set.hasAttribute(HEADER_STAMP_SECONDS))
    data_set.createAttribute(HEADER_STAMP_SECONDS, header.stamp().seconds());
  else
    data_set.getAttribute(HEADER_STAMP_SECONDS).write(header.stamp().seconds());

  if(!data_set.hasAttribute(HEADER_STAMP_NANOS))
    data_set.createAttribute(HEADER_STAMP_NANOS, header.stamp().nanos());
  else
    data_set.getAttribute(HEADER_STAMP_NANOS).write(header.stamp().nanos());

  if(!data_set.hasAttribute(HEADER_FRAME_ID))
    data_set.createAttribute(HEADER_FRAME_ID, header.frame_id());
  else
    data_set.getAttribute(HEADER_FRAME_ID).write(header.frame_id());

  if(!data_set.hasAttribute(HEADER_SEQ))
    data_set.createAttribute(HEADER_SEQ, header.seq());
  else
    data_set.getAttribute(HEADER_SEQ).write(header.seq());

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
  std::shared_ptr<HighFive::DataSet> data_set_ptr;
  HighFive::DataSpace data_space({image.height(), image.step()});

  if(!file.exist(id)) {
    std::cout << "data id " << id << " does nor exist! Creat new dataset in hdf5" << std::endl;
    data_set_ptr = std::make_shared<HighFive::DataSet>(file.createDataSet<uint8_t>(id, data_space));
    data_set_ptr->createAttribute(IMAGE_HEIGHT, image.height());
    data_set_ptr->createAttribute(IMAGE_WIDTH, image.width());
    data_set_ptr->createAttribute(IMAGE_ENCODING, image.encoding());
    data_set_ptr->createAttribute(IMAGE_IS_BIGENDIAN, image.is_bigendian());
    data_set_ptr->createAttribute(IMAGE_STEP, image.step());
  }
  else
  {
    std::cout << "data id " << id << " already exists!" << std::endl;
    data_set_ptr = std::make_shared<HighFive::DataSet>(file.getDataSet(id));
    data_set_ptr->getAttribute(IMAGE_HEIGHT).write(image.height());
    data_set_ptr->getAttribute(IMAGE_WIDTH).write(image.width());
    data_set_ptr->getAttribute(IMAGE_ENCODING).write(image.encoding());
    data_set_ptr->getAttribute(IMAGE_IS_BIGENDIAN).write(image.is_bigendian());
    data_set_ptr->getAttribute(IMAGE_STEP).write(image.step());
  }

  const uint8_t* begin = reinterpret_cast<const uint8_t*>(image.data().c_str());
  std::vector<std::vector<uint8_t>> tmp;
  tmp.resize(image.height());

  for (int i = 0; i < image.height(); i++) {
    const uint8_t* row = begin + i * image.step();
    tmp[i].reserve(image.step());
    std::copy_n(row, image.step(), std::back_inserter(tmp[i]));
  }
  data_set_ptr->write(tmp);
  writeHeaderAttributes(*data_set_ptr, image.header());
  file.flush();
}


std::optional<ag::Image> AGProtoHDF5IO::readImage(const std::string& id)
{
  if(!file.exist(id)) return std::nullopt;

  HighFive::DataSet data_set = file.getDataSet(id);

  ag::Image image;
  data_set.read(image.mutable_data());
  *image.mutable_header() = readHeaderAttributes(data_set);
  return image;
}

} /* namespace ag_proto_hdf5 */
