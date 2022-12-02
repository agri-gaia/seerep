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

  std::string hdf5GroupPath = getHdf5GroupPath(id);
  std::string hdf5DatasetRawDataPath = getHdf5DataSetPath(id);

  std::shared_ptr<HighFive::Group> dataGroupPtr = getHdf5Group(hdf5GroupPath, true);

  HighFive::DataSpace dataSpace({ image.height() * image.step() });
  std::shared_ptr<HighFive::DataSet> dataSetPtr = getHdf5DataSet<uint8_t>(hdf5DatasetRawDataPath, dataSpace);

  writeAttributeToHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CoreImage::HEIGHT, image.height());
  writeAttributeToHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CoreImage::WIDTH, image.width());
  writeAttributeToHdf5<std::string>(*dataSetPtr, seerep_hdf5_core::Hdf5CoreImage::ENCODING, image.encoding()->str());
  writeAttributeToHdf5<bool>(*dataSetPtr, seerep_hdf5_core::Hdf5CoreImage::IS_BIGENDIAN, image.is_bigendian());
  writeAttributeToHdf5<uint32_t>(*dataGroupPtr, seerep_hdf5_core::Hdf5CoreImage::STEP, image.step());

  const uint8_t* dataPtr = image.data()->Data();
  dataSetPtr->write(std::vector<uint8_t>(dataPtr, dataPtr + image.data()->size()));
  writeHeaderAttributes(*dataGroupPtr, image.header());

  writeBoundingBox2DLabeled(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id, image.labels_bb());
  writeLabelsGeneral(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id, image.labels_general());

  m_file->flush();
}

void Hdf5FbImage::writeImageBoundingBox2DLabeled(const std::string& id,
                                                 const seerep::fb::BoundingBoxes2DLabeledStamped& bb2dLabeledStamped)
{
  const std::scoped_lock lock(*m_write_mtx);

  writeBoundingBox2DLabeled(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id, bb2dLabeledStamped.labels_bb());
}

std::optional<flatbuffers::grpc::Message<seerep::fb::Image>> Hdf5FbImage::readImage(const std::string& id,
                                                                                    const bool withoutData)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5GroupPath = getHdf5GroupPath(id);
  std::string hdf5DatasetPath = getHdf5DataSetPath(id);

  try
  {
    checkExists(hdf5GroupPath);
    checkExists(hdf5DatasetPath);
  }
  catch (const std::invalid_argument& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "dataset or data group does not exist";
    return std::nullopt;
  }

  std::shared_ptr<HighFive::DataSet> dataSetPtr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetPath));

  uint32_t step, height, width;
  bool isBigendian;
  std::string encoding;

  try
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "loading attributes";
    height = readAttributeFromHdf5<uint32_t>(id, *dataSetPtr, seerep_hdf5_core::Hdf5CoreImage::HEIGHT);
    width = readAttributeFromHdf5<uint32_t>(id, *dataSetPtr, seerep_hdf5_core::Hdf5CoreImage::WIDTH);
    encoding = readAttributeFromHdf5<std::string>(id, *dataSetPtr, seerep_hdf5_core::Hdf5CoreImage::ENCODING);
    isBigendian = readAttributeFromHdf5<bool>(id, *dataSetPtr, seerep_hdf5_core::Hdf5CoreImage::IS_BIGENDIAN);
    step = readAttributeFromHdf5<uint32_t>(id, *dataSetPtr, seerep_hdf5_core::Hdf5CoreImage::STEP);
  }
  catch (const std::invalid_argument& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << "error: " << e.what();
    return std::nullopt;
  }

  flatbuffers::grpc::MessageBuilder builder;
  std::vector<uint8_t> data;
  flatbuffers::Offset<flatbuffers::Vector<uint8_t>> readDataOffset;
  if (!withoutData)
  {
    dataSetPtr->read(data);
    readDataOffset = builder.CreateVector(data);
  }

  auto headerOffset = readHeaderAttributes(builder, *dataSetPtr, id);

  std::vector<std::string> boundingBoxesLabels;
  std::vector<std::vector<double>> boundingBoxes;
  std::vector<std::string> boundingBoxesInstances;
  readBoundingBoxLabeled(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id, boundingBoxesLabels, boundingBoxes,
                         boundingBoxesInstances);

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
      << "creating the bounding boxes 2d with label fb msgs";
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
  readLabelsGeneral(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id, labelsGeneral, labelsGeneralInstances);

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "creating the label general fb msgs";
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

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << "building the fb image msg";
  flatbuffers::Offset<flatbuffers::String> encodingBuf = builder.CreateString(encoding);

  seerep::fb::ImageBuilder imageBuilder(builder);
  imageBuilder.add_height(height);
  imageBuilder.add_width(width);
  imageBuilder.add_encoding(encodingBuf);
  imageBuilder.add_is_bigendian(isBigendian);
  imageBuilder.add_step(step);

  if (!withoutData)
  {
    imageBuilder.add_data(readDataOffset);
  }
  imageBuilder.add_header(headerOffset);

  imageBuilder.add_labels_bb(bblabeledVectorOffset);
  imageBuilder.add_labels_general(labelsGeneralOffset);

  auto imageOffset = imageBuilder.Finish();
  builder.Finish(imageOffset);
  auto grpcImage = builder.ReleaseMessage<seerep::fb::Image>();
  return grpcImage;
}

const std::string Hdf5FbImage::getHdf5GroupPath(const std::string& id) const
{
  return seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE + "/" + id;
}

const std::string Hdf5FbImage::getHdf5DataSetPath(const std::string& id) const
{
  return getHdf5GroupPath(id) + "/" + seerep_hdf5_core::Hdf5CoreImage::RAWDATA;
}

}  // namespace seerep_hdf5_fb
