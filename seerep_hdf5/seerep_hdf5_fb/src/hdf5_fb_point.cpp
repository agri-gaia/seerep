#include "seerep_hdf5_fb/hdf5_fb_point.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_fb
{
Hdf5FbPoint::Hdf5FbPoint(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx), Hdf5FbGeneral(file, write_mtx)
{
}

void Hdf5FbPoint::writePoint(const std::string& id, const seerep::fb::PointStamped* point)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5DatasetRawDataPath = getHdf5DatasetRawDataPath(id);

  std::shared_ptr<HighFive::DataSet> data_set_ptr;

  std::vector<double> data;
  data.push_back(point->point()->x());
  data.push_back(point->point()->y());
  data.push_back(point->point()->z());

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
    HighFive::DataSpace data_space(3);
    data_set_ptr = std::make_shared<HighFive::DataSet>(
        m_file->createDataSet<double>(hdf5DatasetRawDataPath, HighFive::DataSpace::From(data)));
  }

  data_set_ptr->write(data);
  writeHeaderAttributes(*data_set_ptr, point->header());

  writeAttributeMap(data_set_ptr, point->attribute());

  writeLabels(seerep_hdf5_core::Hdf5CorePoint::HDF5_GROUP_POINT, id, point->labels());

  m_file->flush();
}

void Hdf5FbPoint::writeAdditionalPointAttributes(const seerep::fb::AttributesStamped& attributeStamped)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5DatasetRawDataPath = getHdf5DatasetRawDataPath(attributeStamped.header()->uuid_msgs()->str());
  try
  {
    checkExists(hdf5DatasetRawDataPath);

    std::shared_ptr<HighFive::DataSet> data_set_ptr =
        std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRawDataPath));

    writeAttributeMap(data_set_ptr, attributeStamped.attribute());
  }
  catch (const std::exception& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
        << "loading " << hdf5DatasetRawDataPath << std::endl
        << e.what();
  }
}

std::optional<flatbuffers::grpc::Message<seerep::fb::PointStamped>> Hdf5FbPoint::readPoint(const std::string& id)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5DatasetRawDataPath = getHdf5DatasetRawDataPath(id);
  if (!m_file->exist(hdf5DatasetRawDataPath))
  {
    return std::nullopt;
  }

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "loading " << hdf5DatasetRawDataPath;

  std::shared_ptr<HighFive::DataSet> data_set_ptr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRawDataPath));

  flatbuffers::grpc::MessageBuilder builder;

  auto attributeMapOffset = readAttributeMap(*data_set_ptr, builder);

  std::vector<double> read_data;
  data_set_ptr->read(read_data);
  seerep::fb::PointBuilder pointBuilder(builder);
  pointBuilder.add_x(read_data.at(0));
  pointBuilder.add_y(read_data.at(1));
  pointBuilder.add_z(read_data.at(2));
  auto pointOffset = pointBuilder.Finish();

  auto headerOffset = readHeaderAttributes(builder, *data_set_ptr, id);

  auto labelsOffset = readLabels(seerep_hdf5_core::Hdf5CorePoint::HDF5_GROUP_POINT, id, builder);

  seerep::fb::PointStampedBuilder pointStampedBuilder(builder);

  pointStampedBuilder.add_attribute(attributeMapOffset);
  pointStampedBuilder.add_header(headerOffset);
  pointStampedBuilder.add_labels(labelsOffset);
  pointStampedBuilder.add_point(pointOffset);

  auto pointStampedOffset = pointStampedBuilder.Finish();
  builder.Finish(pointStampedOffset);
  auto grpcPoint = builder.ReleaseMessage<seerep::fb::PointStamped>();
  return grpcPoint;
}

std::string Hdf5FbPoint::getHdf5DatasetRawDataPath(const std::string& id)
{
  return seerep_hdf5_core::Hdf5CorePoint::HDF5_GROUP_POINT + "/" + id + "/" + seerep_hdf5_core::Hdf5CorePoint::RAWDATA;
}

}  // namespace seerep_hdf5_fb
