#include "seerep_hdf5_fb/hdf5_fb_image.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_fb
{
Hdf5FbImage::Hdf5FbImage(std::shared_ptr<HighFive::File>& file,
                         std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx)
  , Hdf5FbGeneral(file, write_mtx)
  , Hdf5CoreImage(file, write_mtx)
{
}

void Hdf5FbImage::writeImage(const std::string& id,
                             const seerep::fb::Image& image)
{
  const std::scoped_lock lock(*m_write_mtx);
  const std::string hdf5GroupPath = getHdf5GroupPath(id);

  HighFive::DataSpace dataSpace(image.height() * image.step());
  std::shared_ptr<HighFive::Group> dataGroup =
      getHdf5Group(hdf5GroupPath, true);

  writeHeaderAttributes(*dataGroup, image.header());

  seerep_hdf5_core::ImageAttributes imageAttributes = {
    image.height(),       image.width(),
    image.step(),         image.encoding()->str(),
    image.is_bigendian(), image.uuid_cameraintrinsics()->str()
  };
  writeImageAttributes(*dataGroup, imageAttributes);

  if (image.data() != nullptr)
  {
    const std::string rawDataPath = getHdf5DataSetPath(id);
    getHdf5DataSet<uint8_t>(rawDataPath, dataSpace)
        ->write(
            std::vector<uint8_t>(image.data()->begin(), image.data()->end()));
  }

  if (image.uri() != nullptr)
  {
    writeAttributeToHdf5<std::string>(
        *dataGroup, seerep_hdf5_core::Hdf5CoreGeneral::DATA_URI,
        image.uri()->str());
  }

  // name is currently ambiguous, use fully qualified name
  seerep_hdf5_fb::Hdf5FbGeneral::writeLabelsFb(
      seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id, image.labels());

  m_file->flush();
}

std::optional<flatbuffers::grpc::Message<seerep::fb::Image>>
Hdf5FbImage::readImage(const std::string& id, const bool withoutData)
{
  const std::scoped_lock lock(*m_write_mtx);
  flatbuffers::grpc::MessageBuilder builder;

  const std::string hdf5GroupPath = getHdf5GroupPath(id);
  if (!exists(hdf5GroupPath))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
        << "[seerep_hdf5_fb] Requested image '" << id << "'  does not exist";
    return std::nullopt;
  }

  std::shared_ptr<HighFive::Group> dataGroup = getHdf5Group(hdf5GroupPath);

  auto imageAttributes = readImageAttributes(*dataGroup);
  auto headerOffset = readHeaderAttributes(builder, *dataGroup, id);
  auto labelsOffset = readLabels(
      seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id, builder);

  flatbuffers::Offset<ByteArrayFb> imageDataOffset;
  const std::string datasetPath = getHdf5DataSetPath(id);

  if (!withoutData && exists(datasetPath))
  {
    std::shared_ptr<HighFive::DataSet> dataSetPtr = getHdf5DataSet(datasetPath);
    std::vector<uint8_t> data;
    data.resize(imageAttributes.height * imageAttributes.width);
    dataSetPtr->read(data);
    imageDataOffset = builder.CreateVector(data);
  }

  std::string dataUri = "";
  flatbuffers::Offset<flatbuffers::String> dataUriOffset;
  if (dataGroup->hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::DATA_URI))
  {
    dataUri = readAttributeFromHdf5<std::string>(
        *dataGroup, seerep_hdf5_core::Hdf5CoreGeneral::DATA_URI, hdf5GroupPath);
    dataUriOffset = builder.CreateString(dataUri);
  }

  auto camintrinsics_offset =
      builder.CreateString(imageAttributes.cameraIntrinsicsUuid);

  auto imageEncodingOffset = builder.CreateString(imageAttributes.encoding);

  seerep::fb::ImageBuilder imageBuilder(builder);
  imageBuilder.add_height(imageAttributes.height);
  imageBuilder.add_width(imageAttributes.width);
  imageBuilder.add_encoding(imageEncodingOffset);
  imageBuilder.add_is_bigendian(imageAttributes.isBigendian);
  imageBuilder.add_uuid_cameraintrinsics(camintrinsics_offset);
  imageBuilder.add_step(imageAttributes.step);
  if (!withoutData)
  {
    imageBuilder.add_data(imageDataOffset);
  }
  if (!dataUri.empty())
  {
    imageBuilder.add_uri(dataUriOffset);
  }
  imageBuilder.add_header(headerOffset);
  if (labelsOffset.has_value())
  {
    imageBuilder.add_labels(labelsOffset.value());
  }

  auto imageOffset = imageBuilder.Finish();
  builder.Finish(imageOffset);
  auto grpcImage = builder.ReleaseMessage<seerep::fb::Image>();
  return grpcImage;
}

}  // namespace seerep_hdf5_fb
