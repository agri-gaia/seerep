#include "seerep-hdf5/ioImage.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5
{
SeerepHDF5IOImage::SeerepHDF5IOImage(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : m_file(file), m_write_mtx(write_mtx), SeerepHDF5IOGeneral(file, write_mtx)
{
}

void SeerepHDF5IOImage::writeImage(const std::string& id, const seerep::Image& image)
{
  std::string hdf5DatasetPath = HDF5_GROUP_IMAGE + "/" + id;
  std::string hdf5DatasetRawDataPath = hdf5DatasetPath + "/" + RAWDATA;

  std::shared_ptr<HighFive::DataSet> data_set_ptr;
  HighFive::DataSpace data_space({ image.height(), image.width(), image.step() / image.width() });

  if (!m_file->exist(hdf5DatasetRawDataPath))
  {
    std::cout << "data id " << hdf5DatasetRawDataPath << " does not exist! Creat new dataset in hdf5" << std::endl;
    data_set_ptr =
        std::make_shared<HighFive::DataSet>(m_file->createDataSet<uint8_t>(hdf5DatasetRawDataPath, data_space));
  }
  else
  {
    std::cout << "data id " << hdf5DatasetRawDataPath << " already exists!" << std::endl;
    data_set_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRawDataPath));
  }

  writeAttribute<uint32_t>(data_set_ptr, HEIGHT, image.height());
  writeAttribute<uint32_t>(data_set_ptr, WIDTH, image.width());
  writeAttribute<std::string>(data_set_ptr, ENCODING, image.encoding());
  writeAttribute<bool>(data_set_ptr, IS_BIGENDIAN, image.is_bigendian());
  writeAttribute<uint32_t>(data_set_ptr, POINT_STEP, image.step());
  writeAttribute<uint32_t>(data_set_ptr, ROW_STEP, image.row_step());

  if (image.encoding() == "rgb8" || image.encoding() == "8UC3")
  {
    writeAttribute<std::string>(data_set_ptr, CLASS, "IMAGE");
    writeAttribute<std::string>(data_set_ptr, "IMAGE_VERSION", "1.2");
    writeAttribute<std::string>(data_set_ptr, "IMAGE_SUBCLASS", "IMAGE_TRUECOLOR");
    writeAttribute<std::string>(data_set_ptr, "INTERLACE_MODE", "INTERLACE_PIXEL");
  }
  else
  {
    deleteAttribute(data_set_ptr, CLASS);
    deleteAttribute(data_set_ptr, "IMAGE_VERSION");
    deleteAttribute(data_set_ptr, "IMAGE_SUBCLASS");
    deleteAttribute(data_set_ptr, "INTERLACE_MODE");
  }

  const uint8_t* begin = reinterpret_cast<const uint8_t*>(image.data().c_str());

  std::vector<std::vector<std::vector<uint8_t>>> tmp;
  tmp.resize(image.height());

  int pixel_step = image.step() / image.width();

  for (int row = 0; row < image.height(); row++)
  {
    tmp.at(row).resize(image.width());
    for (int col = 0; col < image.width(); col++)
    {
      const uint8_t* pxl = begin + row * image.step() + col * pixel_step;
      tmp.at(row).at(col).reserve(pixel_step);
      std::copy_n(pxl, pixel_step, std::back_inserter(tmp.at(row).at(col)));
    }
  }

  data_set_ptr->write(tmp);
  writeHeaderAttributes(*data_set_ptr, image.header());

  writeBoundingBox2DLabeled(HDF5_GROUP_IMAGE, id, image.labels_bb());
  writeLabelsGeneral(HDF5_GROUP_IMAGE, id, image.labels_general());

  m_file->flush();
}

std::optional<seerep::Image> SeerepHDF5IOImage::readImage(const std::string& id)
{
  std::string hdf5DatasetPath = HDF5_GROUP_IMAGE + "/" + id;
  std::string hdf5DatasetRawDataPath = hdf5DatasetPath + "/" + RAWDATA;

  if (!m_file->exist(hdf5DatasetRawDataPath))
    return std::nullopt;

  std::cout << "loading " << hdf5DatasetRawDataPath << std::endl;

  std::shared_ptr<HighFive::DataSet> data_set_ptr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRawDataPath));

  seerep::Image image;

  try
  {
    image.set_height(getAttribute<uint32_t>(id, data_set_ptr, HEIGHT));
    image.set_width(getAttribute<uint32_t>(id, data_set_ptr, WIDTH));
    image.set_encoding(getAttribute<std::string>(id, data_set_ptr, ENCODING));
    image.set_is_bigendian(getAttribute<bool>(id, data_set_ptr, IS_BIGENDIAN));
    image.set_step(getAttribute<uint32_t>(id, data_set_ptr, POINT_STEP));
    image.set_row_step(getAttribute<uint32_t>(id, data_set_ptr, ROW_STEP));
  }
  catch (const std::invalid_argument e)
  {
    std::cout << "error: " << e.what() << std::endl;
    return std::nullopt;
  }
  std::vector<std::vector<std::vector<uint8_t>>> read_data;
  data_set_ptr->read(read_data);

  int pixel_step = image.step() / image.width();

  // TODO: write into protobuf data buffer
  uint8_t data[image.height()][image.width()][pixel_step];

  for (int row = 0; row < image.height(); row++)
  {
    for (int col = 0; col < image.width(); col++)
    {
      std::copy(read_data.at(row).at(col).begin(), read_data.at(row).at(col).end(), data[row][col]);
    }
  }

  // std::cout << "read_data:" << std::endl;
  // int j = 0;
  // for (const auto& i : read_data)
  // {
  //   std::cout << unsigned(i) << ' ';
  //   j++;
  //   // if (j > 50)
  //   // break;
  // }
  image.set_data(data, sizeof(data));

  *image.mutable_header() = readHeaderAttributes(*data_set_ptr);
  auto labelsBB = readBoundingBox2DLabeled(HDF5_GROUP_IMAGE, id);
  if (labelsBB)
  {
    *image.mutable_labels_bb() = labelsBB.value();
  }
  auto labelsGeneral = readLabelsGeneral(HDF5_GROUP_IMAGE, id);
  if (labelsGeneral)
  {
    *image.mutable_labels_general() = labelsGeneral.value();
  }
  return image;
}

} /* namespace seerep_hdf5 */
