#include "seerep-hdf5-pb/hdf5-pb-image.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_pb
{
Hdf5PbImage::Hdf5PbImage(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx), Hdf5CoreImage(file, write_mtx), Hdf5PbGeneral(file, write_mtx)
{
}

void Hdf5PbImage::writeImage(const std::string& id, const seerep::Image& image)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5GroupPath = getHdf5GroupPath(id);
  std::string hdf5DataSetPath = getHdf5DataSetPath(id);

  HighFive::DataSpace dataSpace({ image.height() * image.width() * 3 });
  auto dataGroupPtr = getHdf5Group(hdf5GroupPath);
  auto dataSetPtr = getHdf5DataSet<uint8_t>(hdf5DataSetPath, dataSpace);

  writeHeaderAttributes(*dataGroupPtr, image.header());

  seerep_hdf5_core::ImageAttributes imageAttributes = { image.height(), image.width(), image.step(), image.encoding(),
                                                        image.is_bigendian() };

  writeImageAttributes(id, imageAttributes);

  const uint8_t* arrayStartPtr = reinterpret_cast<const uint8_t*>(image.data().c_str());
  dataSetPtr->write(std::vector<uint8_t>(arrayStartPtr, arrayStartPtr + image.data().size()));

  writeBoundingBox2DLabeled(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id, image.labels_bb());
  seerep_hdf5_pb::Hdf5PbGeneral::writeLabelsGeneral(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id,
                                                    image.labels_general());

  m_file->flush();
}

std::optional<seerep::Image> Hdf5PbImage::readImage(const std::string& id)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5GroupPath = getHdf5GroupPath(id);
  std::string hdf5DataSetPath = getHdf5DataSetPath(id);

  // TODO add logging
  if (!exists(hdf5DataSetPath))
  {
    return std::nullopt;
  }
  else if (!exists(hdf5GroupPath))
  {
    return std::nullopt;
  }

  // read data from hdf5
  auto dataSetPtr = getHdf5DataSet(hdf5DataSetPath);
  auto dataGroupPtr = getHdf5Group(hdf5GroupPath);

  auto header = readHeaderAttributes(*dataGroupPtr, id);

  auto ImageAttributes = readImageAttributes(id);

  auto labelsBB = readBoundingBox2DLabeled(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id);
  auto labelsGeneral = readLabelsGeneral(seerep_hdf5_core::Hdf5CoreImage::HDF5_GROUP_IMAGE, id);

  std::vector<uint8_t> data;
  dataSetPtr->read(data);

  // create pb image message
  seerep::Image image;

  *image.mutable_header() = header;

  image.set_height(ImageAttributes.height);
  image.set_width(ImageAttributes.width);
  image.set_step(ImageAttributes.step);
  image.set_encoding(ImageAttributes.encoding);
  image.set_is_bigendian(ImageAttributes.isBigendian);

  *image.mutable_data() = { data.begin(), data.end() };

  if (labelsBB)
  {
    *image.mutable_labels_bb() = labelsBB.value();
  }
  if (labelsGeneral)
  {
    *image.mutable_labels_general() = labelsGeneral.value();
  }
  return image;
}

} /* namespace seerep_hdf5_pb */
