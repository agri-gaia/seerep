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
                             uint32_t sequence, const py::array& image, const std::vector<GeneralLabel>& general_labels,
                             const std::vector<CategorizedBoundingBoxLabel<2>>& bb_labels)
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

  std::string hdf5_group_path = getHdf5GroupPath(uuid);
  std::string hdf5_dataset_path = getHdf5DataSetPath(uuid);

  HighFive::DataSpace image_data_space({ image_buff_info.shape[0] * image_buff_info.shape[1] * num_channels });
  auto data_droup_ptr = getHdf5Group(hdf5_group_path);

  std::size_t channel_size = 0;  // element (channel data) size in bytes
  if (image.dtype().char_() == 'B')
  {
    // image data is uint8_t
    auto image_dataset_ptr = getHdf5DataSet<uint8_t>(hdf5_dataset_path, image_data_space);
    channel_size = sizeof(uint8_t);

    const auto& typed_image = static_cast<py::array_t<uint8_t>>(image);
    image_dataset_ptr->write(std::vector<uint8_t>(
        typed_image.data(), typed_image.data() + image_buff_info.shape[0] * image_buff_info.shape[1] * num_channels));
  }
  else if (image.dtype().char_() == 'H')
  {
    // image data is uint16_t
    auto image_dataset_ptr = getHdf5DataSet<uint16_t>(hdf5_dataset_path, image_data_space);
    encoding = "rgb16";
    channel_size = sizeof(uint16_t);

    const auto& typed_image = static_cast<py::array_t<uint16_t>>(image);
    image_dataset_ptr->write(std::vector<uint16_t>(
        typed_image.data(), typed_image.data() + image_buff_info.shape[0] * image_buff_info.shape[1] * num_channels));
  }
  else
  {
    throw std::invalid_argument("image data dtype needs to be either 8 or 16 bit unsigned integer");
  }

  // append amount of bits per channel to encoding name
  encoding = encoding + std::to_string(8 * channel_size);

  writeAttributeToHdf5(*data_droup_ptr, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS, seconds);
  writeAttributeToHdf5(*data_droup_ptr, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS, nanos);
  writeAttributeToHdf5(*data_droup_ptr, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID, frame_id);
  writeAttributeToHdf5(*data_droup_ptr, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ, sequence);

  seerep_hdf5_core::ImageAttributes image_attributes = {
    static_cast<uint32_t>(image_buff_info.shape[0]), static_cast<uint32_t>(image_buff_info.shape[1]),
    static_cast<uint32_t>(image_buff_info.shape[1] * num_channels * channel_size), encoding, false
  };

  writeImageAttributes(uuid, image_attributes);

  Hdf5PyGeneral::writeBoundingBoxLabeled(hdf5_group_path, bb_labels);
  Hdf5PyGeneral::writeLabelsGeneral(hdf5_group_path, general_labels);

  m_file->flush();
}

std::tuple<py::array, std::vector<GeneralLabel>, std::vector<CategorizedBoundingBoxLabel<2>>>
Hdf5PyImage::readImage(const std::string& uuid)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5_group_path = getHdf5GroupPath(uuid);
  std::string hdf5_dataset_path = getHdf5DataSetPath(uuid);

  if (!exists(hdf5_dataset_path) || !exists(hdf5_group_path))
  {
    throw std::invalid_argument("image with uuid '" + uuid + "' does not exist");
  }

  // read data from hdf5
  auto image_dataset_ptr = getHdf5DataSet(hdf5_dataset_path);

  auto image_attributes = readImageAttributes(uuid);

  py::array image;

  if (image_attributes.encoding.compare("mono8") == 0 || image_attributes.encoding.compare("rgb8") == 0 ||
      image_attributes.encoding.compare("rgba8") == 0)
  {
    image = py::array_t<uint8_t>(
        { image_attributes.height, image_attributes.width, image_attributes.step / image_attributes.width });

    std::vector<uint8_t> hdf5_data;
    image_dataset_ptr->read(hdf5_data);

    py::buffer_info image_buff_info = image.request();
    uint8_t* image_buff_data = static_cast<uint8_t*>(image_buff_info.ptr);

    for (std::size_t i = 0; i < hdf5_data.size(); i++)
    {
      image_buff_data[i] = hdf5_data[i];
    }
  }
  else if (image_attributes.encoding.compare("mono16") == 0 || image_attributes.encoding.compare("rgb16") == 0 ||
           image_attributes.encoding.compare("rgba16") == 0)
  {
    image = py::array_t<uint16_t>(
        { image_attributes.height, image_attributes.width, image_attributes.step / image_attributes.width / 2 });

    std::vector<uint16_t> hdf5_data;
    image_dataset_ptr->read(hdf5_data);

    py::buffer_info image_buff_info = image.request();
    uint16_t* image_buff_data = static_cast<uint16_t*>(image_buff_info.ptr);

    for (std::size_t i = 0; i < hdf5_data.size(); i++)
    {
      image_buff_data[i] = hdf5_data[i];
    }
  }
  else
  {
    throw std::invalid_argument("unsupported encoding '" + image_attributes.encoding + "'");
  }

  auto general_labels = Hdf5PyGeneral::readLabelsGeneral(hdf5_group_path);
  auto bb_labels = Hdf5PyGeneral::readBoundingBoxLabeled<2>(hdf5_group_path);

  return std::make_tuple(image, general_labels, bb_labels);
}

} /* namespace seerep_hdf5_py */
