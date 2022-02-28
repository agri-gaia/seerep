#include "seerep-hdf5-flatbuffers/image-io-fbs.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5
{
ImageIOFbs::ImageIOFbs(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : GeneralIOFbs(file, write_mtx)
{
}

void ImageIOFbs::writeImage(const std::string& id, const seerep::fb::Image& image)
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
  writeAttribute<std::string>(data_set_ptr, ENCODING, image.encoding()->str());
  writeAttribute<bool>(data_set_ptr, IS_BIGENDIAN, image.is_bigendian());
  writeAttribute<uint32_t>(data_set_ptr, POINT_STEP, image.step());
  writeAttribute<uint32_t>(data_set_ptr, ROW_STEP, image.row_step());

  if (image.encoding()->str() == "rgb8" || image.encoding()->str() == "8UC3")
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

  const uint8_t* begin = reinterpret_cast<const uint8_t*>(image.data());

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
  writeHeaderAttributes(*data_set_ptr, *image.header());

  writeBoundingBox2DLabeled(HDF5_GROUP_IMAGE, id, *image.labels_bb());
  writeLabelsGeneral(HDF5_GROUP_IMAGE, id, *image.labels_general());

  m_file->flush();
}

std::optional<flatbuffers::Offset<seerep::fb::Image>> ImageIOFbs::readImage(const std::string& id)
{
  std::string hdf5DatasetPath = HDF5_GROUP_IMAGE + "/" + id;
  std::string hdf5DatasetRawDataPath = hdf5DatasetPath + "/" + RAWDATA;

  if (!m_file->exist(hdf5DatasetRawDataPath))
    return std::nullopt;

  std::cout << "loading " << hdf5DatasetRawDataPath << std::endl;

  std::shared_ptr<HighFive::DataSet> data_set_ptr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRawDataPath));

  flatbuffers::FlatBufferBuilder builder;
  seerep::fb::ImageBuilder imageBuilder(builder);

  int step, height, width;
  try
  {
    height = getAttribute<uint32_t>(id, data_set_ptr, HEIGHT);
    imageBuilder.add_height(height);
    width = getAttribute<uint32_t>(id, data_set_ptr, WIDTH);
    imageBuilder.add_width(width);
    imageBuilder.add_encoding(builder.CreateString(getAttribute<std::string>(id, data_set_ptr, ENCODING)));
    imageBuilder.add_is_bigendian(getAttribute<bool>(id, data_set_ptr, IS_BIGENDIAN));
    step = getAttribute<uint32_t>(id, data_set_ptr, POINT_STEP);
    imageBuilder.add_step(step);
    imageBuilder.add_row_step(getAttribute<uint32_t>(id, data_set_ptr, ROW_STEP));
  }
  catch (const std::invalid_argument e)
  {
    std::cout << "error: " << e.what() << std::endl;
    return std::nullopt;
  }

  std::vector<uint8_t> read_data;
  data_set_ptr->read(read_data);

  int pixel_step = step / width;

  // // TODO: write into protobuf data buffer
  // uint8_t data[height][width][pixel_step];

  // for (int row = 0; row < height; row++)
  // {
  //   for (int col = 0; col < width; col++)
  //   {
  //     std::copy(read_data.at(row).at(col).begin(), read_data.at(row).at(col).end(), data[row][col]);
  //   }
  // }

  // auto dataFbs = builder.CreateVector(read_data);
  imageBuilder.add_data(builder.CreateVector(read_data));

  imageBuilder.add_header(readHeaderAttributes(*data_set_ptr));

  std::vector<std::string> labels;
  readLabelsGeneral(HDF5_GROUP_IMAGE, id, labels);

  auto vecofstrings = builder.CreateVector<flatbuffers::Offset<flatbuffers::String>>(
      labels.size(),
      [labels](size_t i, flatbuffers::FlatBufferBuilder* b) -> flatbuffers::Offset<flatbuffers::String> {
        return b->CreateSharedString(labels.at(i));
      },
      &builder);

  imageBuilder.add_labels_general(vecofstrings);

  return imageBuilder.Finish();
  // // std::cout << "read_data:" << std::endl;
  // // int j = 0;
  // // for (const auto& i : read_data)
  // // {
  // //   std::cout << unsigned(i) << ' ';
  // //   j++;
  // //   // if (j > 50)
  // //   // break;
  // // }
  // image.set_data(data, sizeof(data));

  // *image.mutable_header() = readHeaderAttributes(*data_set_ptr);
  // auto labelsBB = readBoundingBox2DLabeled(HDF5_GROUP_IMAGE, id);
  // if (labelsBB)
  // {
  //   *image.mutable_labels_bb() = labelsBB.value();
  // }
  // if (labelsGeneral)
  // {
  //   *image.mutable_labels_general() = labelsGeneral.value();
  // }
  // return image;
}

} /* namespace seerep_hdf5 */
