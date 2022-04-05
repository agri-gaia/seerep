#include "seerep-hdf5-fb/hdf5-fb-image.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_fb
{
Hdf5FbImage::Hdf5FbImage(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5FbGeneral(file, write_mtx)
{
}

void Hdf5FbImage::writeImage(const std::string& id, const seerep::fb::Image& image)
{
  const std::scoped_lock lock(*m_write_mtx);

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

  flatbuffers::VectorIterator begin = image.data()->cbegin();

  std::vector<std::vector<std::vector<uint8_t>>> tmp;
  tmp.resize(image.height());

  int pixel_step = image.step() / image.width();

  for (int row = 0; row < image.height(); row++)
  {
    tmp.at(row).resize(image.width());
    for (int col = 0; col < image.width(); col++)
    {
      // for (int channel = 0; channel < pixel_step; channel++)
      // {
      //   tmp.at(row).at(col).push_back(*(begin + row * image.step() + col * pixel_step + channel));
      // }

      //   const uint8_t* pxl = begin + row * image.step() + col * pixel_step;
      // // tmp.at(row).at(col).reserve(pixel_step);
      std::copy_n(begin + row * image.step() + col * pixel_step, pixel_step, std::back_inserter(tmp.at(row).at(col)));
    }
  }

  data_set_ptr->write(tmp);
  writeHeaderAttributes(*data_set_ptr, *image.header());

  writeBoundingBox2DLabeled(HDF5_GROUP_IMAGE, id, image.labels_bb());
  writeLabelsGeneral(HDF5_GROUP_IMAGE, id, image.labels_general());

  m_file->flush();
}

void Hdf5FbImage::writeImageBoundingBox2DLabeled(const std::string& id,
                                                 const seerep::fb::BoundingBoxes2DLabeledStamped& bb2dLabeledStamped)
{
  const std::scoped_lock lock(*m_write_mtx);

  writeBoundingBox2DLabeled(HDF5_GROUP_IMAGE, id, bb2dLabeledStamped.labels_bb());
}

void Hdf5FbImage::readImage(const std::string& id, const std::string& projectuuid,
                            grpc::ServerWriter<flatbuffers::grpc::Message<seerep::fb::Image>>* const writer)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5DatasetPath = HDF5_GROUP_IMAGE + "/" + id;
  std::string hdf5DatasetRawDataPath = hdf5DatasetPath + "/" + RAWDATA;

  if (!m_file->exist(hdf5DatasetRawDataPath))
    return;

  std::cout << "loading " << hdf5DatasetRawDataPath << std::endl;

  std::shared_ptr<HighFive::DataSet> data_set_ptr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRawDataPath));

  flatbuffers::grpc::MessageBuilder builder;

  uint32_t step, height, width, rowStep;
  flatbuffers::Offset<flatbuffers::String> encoding;
  bool isBigendian;
  try
  {
    height = getAttribute<uint32_t>(id, data_set_ptr, HEIGHT);
    width = getAttribute<uint32_t>(id, data_set_ptr, WIDTH);
    encoding = builder.CreateString(getAttribute<std::string>(id, data_set_ptr, ENCODING));
    isBigendian = getAttribute<bool>(id, data_set_ptr, IS_BIGENDIAN);
    step = getAttribute<uint32_t>(id, data_set_ptr, POINT_STEP);
    rowStep = getAttribute<uint32_t>(id, data_set_ptr, ROW_STEP);
  }
  catch (const std::invalid_argument e)
  {
    std::cout << "error: " << e.what() << std::endl;
    return;
  }

  std::vector<std::vector<std::vector<uint8_t>>> read_data;
  data_set_ptr->read(read_data);

  int pixel_step = step / width;
  // uint8_t data[height][width][pixel_step];
  std::vector<uint8_t> data;
  data.reserve(height * width * pixel_step);
  for (int row = 0; row < height; row++)
  {
    for (int col = 0; col < width; col++)
    {
      // std::copy(read_data.at(row).at(col).begin(), read_data.at(row).at(col).end(), data[row]);
      data.insert(data.end(), std::make_move_iterator(read_data.at(row).at(col).begin()),
                  std::make_move_iterator(read_data.at(row).at(col).end()));
    }
  }

  auto readDataOffset = builder.CreateVector(data);
  auto headerOffset = readHeaderAttributes(*data_set_ptr, projectuuid, id, builder);

  std::vector<std::string> boundingBoxesLabels;
  std::vector<std::vector<double>> boundingBoxes;
  readBoundingBox2DLabeled(HDF5_GROUP_IMAGE, id, boundingBoxesLabels, boundingBoxes);

  std::vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeled>> bblabeledVector;
  for (int i = 0; i < boundingBoxes.size(); i++)
  {
    auto labelOffset = builder.CreateString(boundingBoxesLabels.at(i));

    auto pointMin = seerep::fb::CreatePoint2D(builder, boundingBoxes.at(i).at(0), boundingBoxes.at(i).at(1));
    auto pointMax = seerep::fb::CreatePoint2D(builder, boundingBoxes.at(i).at(2), boundingBoxes.at(i).at(3));

    seerep::fb::Boundingbox2DBuilder bbBuilder(builder);
    bbBuilder.add_point_min(pointMin);
    bbBuilder.add_point_max(pointMax);
    auto bb = bbBuilder.Finish();

    seerep::fb::BoundingBox2DLabeledBuilder bblabeledBuilder(builder);
    bblabeledBuilder.add_bounding_box(bb);
    bblabeledBuilder.add_label(labelOffset);

    bblabeledVector.push_back(bblabeledBuilder.Finish());
  }
  auto bblabeledVectorOffset = builder.CreateVector(bblabeledVector);

  std::vector<std::string> labelsGeneral;
  readLabelsGeneral(HDF5_GROUP_IMAGE, id, labelsGeneral);

  auto labelsGeneralOffset = builder.CreateVector<flatbuffers::Offset<flatbuffers::String>>(
      labelsGeneral.size(),
      [labelsGeneral](size_t i, flatbuffers::FlatBufferBuilder* b) -> flatbuffers::Offset<flatbuffers::String> {
        return b->CreateSharedString(labelsGeneral.at(i));
      },
      &builder);

  seerep::fb::ImageBuilder imageBuilder(builder);
  imageBuilder.add_height(height);
  imageBuilder.add_width(width);
  imageBuilder.add_encoding(encoding);
  imageBuilder.add_is_bigendian(isBigendian);
  imageBuilder.add_step(step);
  imageBuilder.add_row_step(rowStep);

  imageBuilder.add_data(readDataOffset);
  imageBuilder.add_header(headerOffset);

  imageBuilder.add_labels_bb(bblabeledVectorOffset);
  imageBuilder.add_labels_general(labelsGeneralOffset);

  auto imageOffset = imageBuilder.Finish();
  builder.Finish(imageOffset);

  writer->Write(builder.ReleaseMessage<seerep::fb::Image>());
}

}  // namespace seerep_hdf5_fb
