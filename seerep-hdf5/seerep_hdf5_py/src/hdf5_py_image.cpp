#include "seerep_hdf5_py/hdf5_py_image.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_py
{

Hdf5PyImage::Hdf5PyImage(Hdf5FileWrapper& hdf5_file)
  : Hdf5CoreGeneral(hdf5_file.getFile(), hdf5_file.getMutex())
  , Hdf5CoreImage(hdf5_file.getFile(), hdf5_file.getMutex())
  , Hdf5PyGeneral(hdf5_file)
{
}

std::vector<std::string> Hdf5PyImage::getImages()
{
  const std::scoped_lock lock(*m_write_mtx);

  if (!m_file->exist(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE))
  {
    return std::vector<std::string>();
  }
  const HighFive::Group& cloudsGroup = m_file->getGroup(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE);
  return cloudsGroup.listObjectNames();
}

void Hdf5PyImage::writeImage(const std::string& uuid, const std::string& frameId, int64_t seconds, int32_t nanos,
                             uint32_t sequence, const py::array& image, const std::vector<GeneralLabel>& generalLabels,
                             const std::vector<CategorizedBoundingBoxLabel<2>>& bbLabels)
{
  const std::scoped_lock lock(*m_write_mtx);

  py::buffer_info imageBuffInfo = image.request();

  std::size_t numChannels = 1;
  if (imageBuffInfo.shape.size() > 2)
  {
    numChannels = imageBuffInfo.shape[2];
  }

  std::string encoding = "";
  if (numChannels == 1)
  {
    encoding = "mono";
  }
  else if (numChannels == 3)
  {
    encoding = "rgb";
  }
  else if (numChannels == 4)
  {
    encoding = "rgba";
  }
  else
  {
    throw std::invalid_argument("image must have either 1 (MONO), 3 (RGB) or 4 (RGBA) channels");
  }

  std::string hdf5GroupPath = getHdf5GroupPath(uuid);
  std::string hdf5DatasetPath = getHdf5DataSetPath(uuid);

  HighFive::DataSpace imageDataSpace({ imageBuffInfo.shape[0] * imageBuffInfo.shape[1] * numChannels });
  auto dataGroupPtr = getHdf5Group(hdf5GroupPath);

  std::size_t channelSize = 0;  // element (channel data) size in bytes
  if (image.dtype().equal(py::dtype::of<uint8_t>()))
  {
    // image data is uint8_t
    auto imageDatasetPtr = getHdf5DataSet<uint8_t>(hdf5DatasetPath, imageDataSpace);
    channelSize = sizeof(uint8_t);

    const auto& typedImage = static_cast<py::array_t<uint8_t>>(image);
    imageDatasetPtr->write(std::vector<uint8_t>(
        typedImage.data(), typedImage.data() + imageBuffInfo.shape[0] * imageBuffInfo.shape[1] * numChannels));
  }
  else if (image.dtype().equal(py::dtype::of<uint16_t>()))
  {
    // image data is uint16_t
    auto imageDatasetPtr = getHdf5DataSet<uint16_t>(hdf5DatasetPath, imageDataSpace);
    encoding = "rgb16";
    channelSize = sizeof(uint16_t);

    const auto& typedImage = static_cast<py::array_t<uint16_t>>(image);
    imageDatasetPtr->write(std::vector<uint16_t>(
        typedImage.data(), typedImage.data() + imageBuffInfo.shape[0] * imageBuffInfo.shape[1] * numChannels));
  }
  else
  {
    throw std::invalid_argument("image data dtype needs to be either 8 or 16 bit unsigned integer");
  }

  // append amount of bits per channel to encoding name
  encoding = encoding + std::to_string(8 * channelSize);

  writeAttributeToHdf5(*dataGroupPtr, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS, seconds);
  writeAttributeToHdf5(*dataGroupPtr, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS, nanos);
  writeAttributeToHdf5(*dataGroupPtr, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID, frameId);
  writeAttributeToHdf5(*dataGroupPtr, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ, sequence);

  seerep_hdf5_core::ImageAttributes image_attributes = {
    static_cast<uint32_t>(imageBuffInfo.shape[0]), static_cast<uint32_t>(imageBuffInfo.shape[1]),
    static_cast<uint32_t>(imageBuffInfo.shape[1] * numChannels * channelSize), encoding, false
  };

  writeImageAttributes(uuid, image_attributes);

  Hdf5PyGeneral::writeBoundingBoxLabeled(hdf5GroupPath, bbLabels);
  Hdf5PyGeneral::writeLabelsGeneral(hdf5GroupPath, generalLabels);

  m_file->flush();
}

std::tuple<py::array, std::vector<GeneralLabel>, std::vector<CategorizedBoundingBoxLabel<2>>>
Hdf5PyImage::readImage(const std::string& uuid)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5GroupPath = getHdf5GroupPath(uuid);
  std::string hdf5DatasetPath = getHdf5DataSetPath(uuid);

  if (!exists(hdf5DatasetPath) || !exists(hdf5GroupPath))
  {
    throw std::invalid_argument("image with uuid '" + uuid + "' does not exist");
  }

  // read data from hdf5
  auto imageDatasetPtr = getHdf5DataSet(hdf5DatasetPath);

  auto imageAttributes = readImageAttributes(uuid);

  py::array image;

  if (imageAttributes.encoding.compare("mono8") == 0 || imageAttributes.encoding.compare("rgb8") == 0 ||
      imageAttributes.encoding.compare("rgba8") == 0)
  {
    image = py::array_t<uint8_t>(
        { imageAttributes.height, imageAttributes.width, imageAttributes.step / imageAttributes.width });

    std::vector<uint8_t> hdf5Data;
    imageDatasetPtr->read(hdf5Data);

    py::buffer_info imageBuffInfo = image.request();
    uint8_t* imageBuffData = static_cast<uint8_t*>(imageBuffInfo.ptr);

    for (std::size_t i = 0; i < hdf5Data.size(); i++)
    {
      imageBuffData[i] = hdf5Data[i];
    }
  }
  else if (imageAttributes.encoding.compare("mono16") == 0 || imageAttributes.encoding.compare("rgb16") == 0 ||
           imageAttributes.encoding.compare("rgba16") == 0)
  {
    image = py::array_t<uint16_t>(
        { imageAttributes.height, imageAttributes.width, imageAttributes.step / imageAttributes.width / 2 });

    std::vector<uint16_t> hdf5Data;
    imageDatasetPtr->read(hdf5Data);

    py::buffer_info imageBuffInfo = image.request();
    uint16_t* imageBuffData = static_cast<uint16_t*>(imageBuffInfo.ptr);

    for (std::size_t i = 0; i < hdf5Data.size(); i++)
    {
      imageBuffData[i] = hdf5Data[i];
    }
  }
  else
  {
    throw std::invalid_argument("unsupported encoding '" + imageAttributes.encoding + "'");
  }

  auto generalLabels = Hdf5PyGeneral::readLabelsGeneral(hdf5GroupPath);
  auto bbLabels = Hdf5PyGeneral::readBoundingBoxLabeled<2>(hdf5GroupPath);

  return std::make_tuple(image, generalLabels, bbLabels);
}

} /* namespace seerep_hdf5_py */
