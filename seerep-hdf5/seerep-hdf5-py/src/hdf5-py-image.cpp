#include "seerep-hdf5-py/hdf5-py-image.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_py
{

Hdf5PyImage::Hdf5PyImage(Hdf5FileWrapper& hdf5_file)
  : Hdf5CoreGeneral(hdf5_file.getFile(), hdf5_file.getMutex())
  , Hdf5CoreImage(hdf5_file.getFile(), hdf5_file.getMutex())
  , Hdf5PyGeneral(hdf5_file)
{
}

void Hdf5PyImage::writeImage(const std::string& uuid, const std::string& frame_id, int64_t seconds, int32_t nanos,
                             uint32_t sequence, const py::array& image)
{
  const std::scoped_lock lock(*m_write_mtx);

  py::buffer_info image_buff_info = image.request();

  std::size_t num_channels = 1;
  if (image_buff_info.shape.size() > 2)
  {
    num_channels = image_buff_info.shape[2];
  }

  std::string encoding = "";
  if (num_channels == 1)
  {
    encoding = "mono";
  }
  else if (num_channels == 3)
  {
    encoding = "rgb";
  }
  else if (num_channels == 4)
  {
    encoding = "rgba";
  }
  else
  {
    throw std::invalid_argument("image must have either 1 (MONO), 3 (RGB) or 4 (RGBA) channels");
  }

  std::string hdf5GroupPath = getHdf5GroupPath(uuid);
  std::string hdf5DataSetPath = getHdf5DataSetPath(uuid);

  HighFive::DataSpace dataSpace({ image_buff_info.shape[0] * image_buff_info.shape[1] * num_channels });
  auto dataGroupPtr = getHdf5Group(hdf5GroupPath);

  std::size_t channel_size = 0;  // element (channel data) size in bytes
  if (image.dtype().char_() == 'B')
  {
    // image data is uint8_t
    auto dataSetPtr = getHdf5DataSet<uint8_t>(hdf5DataSetPath, dataSpace);
    channel_size = sizeof(uint8_t);

    const auto& typed_image = static_cast<py::array_t<uint8_t>>(image);
    dataSetPtr->write(std::vector<uint8_t>(
        typed_image.data(), typed_image.data() + image_buff_info.shape[0] * image_buff_info.shape[1] * num_channels));
  }
  else if (image.dtype().char_() == 'H')
  {
    // image data is uint16_t
    auto dataSetPtr = getHdf5DataSet<uint16_t>(hdf5DataSetPath, dataSpace);
    encoding = "rgb16";
    channel_size = sizeof(uint16_t);

    const auto& typed_image = static_cast<py::array_t<uint16_t>>(image);
    dataSetPtr->write(std::vector<uint16_t>(
        typed_image.data(), typed_image.data() + image_buff_info.shape[0] * image_buff_info.shape[1] * num_channels));
  }
  else
  {
    throw std::invalid_argument("image data dtype needs to be either 8 or 16 bit unsigned integer");
  }

  // append amount of bits per channel to encoding name
  encoding = encoding + std::to_string(8 * channel_size);

  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(
      *dataGroupPtr, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS, seconds);
  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(*dataGroupPtr,
                                                          seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS, nanos);
  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(*dataGroupPtr,
                                                          seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID, frame_id);
  seerep_hdf5_core::Hdf5CoreGeneral::writeAttributeToHdf5(*dataGroupPtr, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ,
                                                          sequence);

  seerep_hdf5_core::ImageAttributes imageAttributes = {
    static_cast<uint32_t>(image_buff_info.shape[0]), static_cast<uint32_t>(image_buff_info.shape[1]),
    static_cast<uint32_t>(image_buff_info.shape[1] * num_channels * channel_size), encoding, false
  };

  writeImageAttributes(uuid, imageAttributes);

  // TODO: write labels

  // writeBoundingBox2DLabeled(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id, image.labels_bb());
  // seerep_hdf5_pb::Hdf5PbGeneral::writeLabelsGeneral(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id,
  //                                                   image.labels_general());

  m_file->flush();
}

py::array Hdf5PyImage::readImage(const std::string& uuid)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5GroupPath = getHdf5GroupPath(uuid);
  std::string hdf5DataSetPath = getHdf5DataSetPath(uuid);

  // TODO add logging
  if (!exists(hdf5DataSetPath) || !exists(hdf5GroupPath))
  {
    throw std::invalid_argument("image with uuid '" + uuid + "' does not exist");
  }

  // read data from hdf5
  auto dataSetPtr = getHdf5DataSet(hdf5DataSetPath);
  auto dataGroupPtr = getHdf5Group(hdf5GroupPath);

  auto imageAttributes = readImageAttributes(uuid);

  py::array image;

  if (imageAttributes.encoding.compare("mono8") == 0 || imageAttributes.encoding.compare("rgb8") == 0 ||
      imageAttributes.encoding.compare("rgba8") == 0)
  {
    image = py::array_t<uint8_t>(
        { imageAttributes.height, imageAttributes.width, imageAttributes.step / imageAttributes.width });

    std::vector<uint8_t> hdf5_data;
    dataSetPtr->read(hdf5_data);

    py::buffer_info image_buff_info = image.request();
    uint8_t* image_buff_data = static_cast<uint8_t*>(image_buff_info.ptr);

    for (std::size_t i = 0; i < hdf5_data.size(); i++)
    {
      image_buff_data[i] = hdf5_data[i];
    }
  }
  else if (imageAttributes.encoding.compare("mono16") == 0 || imageAttributes.encoding.compare("rgb16") == 0 ||
           imageAttributes.encoding.compare("rgba16") == 0)
  {
    image = py::array_t<uint16_t>(
        { imageAttributes.height, imageAttributes.width, imageAttributes.step / imageAttributes.width / 2 });

    std::vector<uint16_t> hdf5_data;
    dataSetPtr->read(hdf5_data);

    py::buffer_info image_buff_info = image.request();
    uint16_t* image_buff_data = static_cast<uint16_t*>(image_buff_info.ptr);

    for (std::size_t i = 0; i < hdf5_data.size(); i++)
    {
      image_buff_data[i] = hdf5_data[i];
    }
  }
  else
  {
    throw std::invalid_argument("unsupported encoding '" + imageAttributes.encoding + "'");
  }

  return image;
}

} /* namespace seerep_hdf5_py */
