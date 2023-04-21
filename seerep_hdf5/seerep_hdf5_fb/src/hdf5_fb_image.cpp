#include "seerep_hdf5_fb/hdf5_fb_image.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_fb
{
Hdf5FbImage::Hdf5FbImage(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx), Hdf5FbGeneral(file, write_mtx), Hdf5CoreImage(file, write_mtx)
{
}

void Hdf5FbImage::writeImage(const std::string& id, const seerep::fb::Image& image)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5GroupPath = getHdf5GroupPath(id);
  std::string hdf5DatasetRawDataPath = getHdf5DataSetPath(id);

  HighFive::DataSpace dataSpace(image.height() * image.step());
  auto dataGroupPtr = getHdf5Group(hdf5GroupPath, true);
  auto dataSetPtr = getHdf5DataSet<uint8_t>(hdf5DatasetRawDataPath, dataSpace);

  writeHeaderAttributes(*dataGroupPtr, image.header());

  seerep_hdf5_core::ImageAttributes imageAttributes = { image.height(),       image.width(),
                                                        image.step(),         image.encoding()->str(),
                                                        image.is_bigendian(), image.uuid_cameraintrinsics()->str() };
  writeImageAttributes(id, imageAttributes);

  const uint8_t* arrayStartPtr = image.data()->Data();
  // use pointers from start and end of the array as iterators
  dataSetPtr->write(std::vector<uint8_t>(arrayStartPtr, arrayStartPtr + image.data()->size()));

  writeBoundingBox2DLabeled(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id, image.labels_bb());
  // name is currently ambiguous, use fully qualified name
  seerep_hdf5_fb::Hdf5FbGeneral::writeLabelsGeneral(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id,
                                                    image.labels_general());

  m_file->flush();
}

void Hdf5FbImage::writeImageBoundingBox2DLabeled(
    const std::string& id,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeledWithCategory>>* bb2DLabeledWithCategory)
{
  const std::scoped_lock lock(*m_write_mtx);

  writeBoundingBox2DLabeled(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id, bb2DLabeledWithCategory);
}

std::optional<flatbuffers::grpc::Message<seerep::fb::Image>> Hdf5FbImage::readImage(const std::string& id,
                                                                                    const bool withoutData)
{
  const std::scoped_lock lock(*m_write_mtx);
  flatbuffers::grpc::MessageBuilder builder;

  std::string hdf5GroupPath = getHdf5GroupPath(id);
  std::string hdf5DatasetPath = getHdf5DataSetPath(id);

  if (!exists(hdf5DatasetPath))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << "DataSet: " << id << " does not exist";
    return std::nullopt;
  }
  else if (!exists(hdf5GroupPath))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << "DataGroup: " << id << " does not exist";
    return std::nullopt;
  }

  // read all data from hdf5 and if possible convert into flatbuffers objects

  auto imageAttributes = readImageAttributes(id);

  auto dataSetPtr = getHdf5DataSet(hdf5DatasetPath);
  auto dataGroupPtr = getHdf5Group(hdf5GroupPath);

  auto headerOffset = readHeaderAttributes(builder, *dataGroupPtr, id);
  auto boxes2DLabeledOffset =
      readBoundingBoxes2DLabeled(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id, builder);
  auto generalLabelsOffset = readGeneralLabels(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id, builder);
  auto encodingStringOffset = builder.CreateString(imageAttributes.encoding);

  std::vector<uint8_t> data;
  flatbuffers::Offset<ByteArrayFb> imageDataOffset;
  if (!withoutData)
  {
    data.resize(imageAttributes.height * imageAttributes.width);
    dataSetPtr->read(data);
  }
  imageDataOffset = builder.CreateVector(data);

  // construct flatbuffers image message

  seerep::fb::ImageBuilder imageBuilder(builder);
  imageBuilder.add_height(imageAttributes.height);
  imageBuilder.add_width(imageAttributes.width);
  imageBuilder.add_encoding(encodingStringOffset);
  imageBuilder.add_is_bigendian(imageAttributes.isBigendian);
  imageBuilder.add_step(imageAttributes.step);
  if (!withoutData)
  {
    imageBuilder.add_data(imageDataOffset);
  }
  imageBuilder.add_header(headerOffset);
  imageBuilder.add_labels_bb(boxes2DLabeledOffset);
  imageBuilder.add_labels_general(generalLabelsOffset);

  auto imageOffset = imageBuilder.Finish();
  builder.Finish(imageOffset);
  auto grpcImage = builder.ReleaseMessage<seerep::fb::Image>();
  return grpcImage;
}

}  // namespace seerep_hdf5_fb
