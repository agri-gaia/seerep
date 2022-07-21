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

  try
  {
    checkExists(hdf5DatasetRawDataPath);
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "data id " << hdf5DatasetRawDataPath << " already exists!";
    data_set_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRawDataPath));
  }
  catch (std::invalid_argument const& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "data id " << hdf5DatasetRawDataPath << " does not exist! Creat new dataset in hdf5";
    data_set_ptr =
        std::make_shared<HighFive::DataSet>(m_file->createDataSet<uint8_t>(hdf5DatasetRawDataPath, data_space));
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

  for (uint32_t row = 0; row < image.height(); row++)
  {
    tmp.at(row).resize(image.width());
    for (uint32_t col = 0; col < image.width(); col++)
    {
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

std::optional<flatbuffers::grpc::Message<seerep::fb::Image>> Hdf5FbImage::readImage(const std::string& id)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5DatasetPath = HDF5_GROUP_IMAGE + "/" + id;
  std::string hdf5DatasetRawDataPath = hdf5DatasetPath + "/" + RAWDATA;

  if (!m_file->exist(hdf5DatasetRawDataPath))
    return std::nullopt;

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "loading " << hdf5DatasetRawDataPath;

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
  catch (const std::invalid_argument& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << "error: " << e.what();
    return std::nullopt;
  }

  std::vector<std::vector<std::vector<uint8_t>>> read_data;
  data_set_ptr->read(read_data);

  int pixel_step = step / width;
  std::vector<uint8_t> data;
  data.reserve(height * width * pixel_step);
  for (uint32_t row = 0; row < height; row++)
  {
    for (uint32_t col = 0; col < width; col++)
    {
      data.insert(data.end(), std::make_move_iterator(read_data.at(row).at(col).begin()),
                  std::make_move_iterator(read_data.at(row).at(col).end()));
    }
  }

  auto readDataOffset = builder.CreateVector(data);
  auto headerOffset = readHeaderAttributes(*data_set_ptr, id, builder);

  std::vector<std::string> boundingBoxesLabels;
  std::vector<std::vector<double>> boundingBoxes;
  std::vector<std::string> boundingBoxesInstances;
  readBoundingBox2DLabeled(HDF5_GROUP_IMAGE, id, boundingBoxesLabels, boundingBoxes, boundingBoxesInstances);

  std::vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeled>> bblabeledVector;
  for (long unsigned int i = 0; i < boundingBoxes.size(); i++)
  {
    auto InstanceOffset = builder.CreateString(boundingBoxesInstances.at(i));
    auto labelOffset = builder.CreateString(boundingBoxesLabels.at(i));

    seerep::fb::LabelWithInstanceBuilder labelBuilder(builder);
    labelBuilder.add_instanceUuid(InstanceOffset);
    labelBuilder.add_label(labelOffset);
    auto labelWithInstanceOffset = labelBuilder.Finish();

    auto pointMin = seerep::fb::CreatePoint2D(builder, boundingBoxes.at(i).at(0), boundingBoxes.at(i).at(1));
    auto pointMax = seerep::fb::CreatePoint2D(builder, boundingBoxes.at(i).at(2), boundingBoxes.at(i).at(3));

    seerep::fb::Boundingbox2DBuilder bbBuilder(builder);
    bbBuilder.add_point_min(pointMin);
    bbBuilder.add_point_max(pointMax);
    auto bb = bbBuilder.Finish();

    seerep::fb::BoundingBox2DLabeledBuilder bblabeledBuilder(builder);
    bblabeledBuilder.add_bounding_box(bb);
    bblabeledBuilder.add_labelWithInstance(labelWithInstanceOffset);

    bblabeledVector.push_back(bblabeledBuilder.Finish());
  }
  auto bblabeledVectorOffset = builder.CreateVector(bblabeledVector);

  std::vector<std::string> labelsGeneral;
  std::vector<std::string> labelsGeneralInstances;
  readLabelsGeneral(HDF5_GROUP_IMAGE, id, labelsGeneral, labelsGeneralInstances);

  std::vector<flatbuffers::Offset<seerep::fb::LabelWithInstance>> labelGeneralVector;
  labelGeneralVector.reserve(labelsGeneral.size());
  for (long unsigned int i = 0; i < labelsGeneral.size(); i++)
  {
    auto labelOffset = builder.CreateString(labelsGeneral.at(i));
    auto instanceOffset = builder.CreateString(labelsGeneralInstances.at(i));

    seerep::fb::LabelWithInstanceBuilder labelBuilder(builder);
    labelBuilder.add_label(labelOffset);
    labelBuilder.add_instanceUuid(instanceOffset);
    labelGeneralVector.push_back(labelBuilder.Finish());
  }

  auto labelsGeneralOffset =
      builder.CreateVector<flatbuffers::Offset<seerep::fb::LabelWithInstance>>(labelGeneralVector);

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
  auto grpcImage = builder.ReleaseMessage<seerep::fb::Image>();
  return grpcImage;
}

}  // namespace seerep_hdf5_fb
