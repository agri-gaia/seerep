#include <filesystem>

namespace seerep_hdf5_core
{
template <typename T, class C>
T Hdf5CoreGeneral::readAttributeFromHdf5(const std::string& id, const HighFive::AnnotateTraits<C>& object,
                                         std::string attributeField)
{
  T attributeValue;
  if (object.hasAttribute(attributeField))
  {
    object.getAttribute(attributeField).read(attributeValue);
  }
  else
  {
    throw std::invalid_argument("id " + id + " has no attribute " + attributeField);
  }
  return attributeValue;
}

template <typename T, class C>
void Hdf5CoreGeneral::writeAttributeToHdf5(HighFive::AnnotateTraits<C>& object, std::string attributeField,
                                           T attributeValue)
{
  if (object.hasAttribute(attributeField))
  {
    object.getAttribute(attributeField).write(attributeValue);
  }
  else
  {
    object.createAttribute(attributeField, attributeValue);
  }
}

template <class C>
std::string Hdf5CoreGeneral::readFrameId(const std::string& uuid, const HighFive::AnnotateTraits<C>& object,
                                         const std::string& frame_field)
{
  return tf2_frame_id(readAttributeFromHdf5<std::string>(uuid, object, frame_field));
}

template <class C>
void Hdf5CoreGeneral::writeFrameId(HighFive::AnnotateTraits<C>& object, const std::string& frame_field,
                                   const std::string& frame_id)
{
  writeAttributeToHdf5<std::string>(object, frame_field, tf2_frame_id(frame_id));
}

template <class T>
std::shared_ptr<HighFive::DataSet> Hdf5CoreGeneral::getHdf5DataSet(const std::string& hdf5DataSetPath,
                                                                   HighFive::DataSpace& dataSpace)
{
  HighFive::DataSetCreateProps emptyProps;
  return getHdf5DataSet<T>(hdf5DataSetPath, dataSpace, emptyProps);
}

template <class T>
std::shared_ptr<HighFive::DataSet> Hdf5CoreGeneral::getHdf5DataSet(const std::string& hdf5DataSetPath,
                                                                   HighFive::DataSpace& dataSpace,
                                                                   HighFive::DataSetCreateProps& createProps)
{
  try
  {
    checkExists(hdf5DataSetPath);
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "hdf5 group" << hdf5DataSetPath << " already exists!";
    return std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DataSetPath));
  }
  catch (std::invalid_argument const& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace)
        << "hdf5 group " << hdf5DataSetPath << " does not exist! Creating a new group";
    return std::make_shared<HighFive::DataSet>(m_file->createDataSet<T>(hdf5DataSetPath, dataSpace, createProps));
  }
}

template <class T>
void Hdf5CoreGeneral::writeHeader(HighFive::AnnotateTraits<T>& object, seerep_core_msgs::Header header)
{
  Hdf5CoreGeneral::writeAttributeToHdf5<int32_t>(object, HEADER_STAMP_SECONDS, header.timestamp.seconds);
  Hdf5CoreGeneral::writeAttributeToHdf5<int32_t>(object, HEADER_STAMP_NANOS, header.timestamp.nanos);
  Hdf5CoreGeneral::writeFrameId(object, HEADER_FRAME_ID, header.frameId);
  Hdf5CoreGeneral::writeAttributeToHdf5<uint32_t>(object, HEADER_SEQ, header.sequence);
}

template <class T>
void Hdf5CoreGeneral::readHeader(const std::string& id, HighFive::AnnotateTraits<T>& object,
                                 seerep_core_msgs::Header& header)
{
  header.uuidData = boost::lexical_cast<boost::uuids::uuid>(id);

  std::string uuidproject_str = std::filesystem::path(m_file->getName()).filename().stem();
  header.uuidProject = boost::lexical_cast<boost::uuids::uuid>(uuidproject_str);

  header.timestamp.seconds = Hdf5CoreGeneral::readAttributeFromHdf5<int32_t>(id, object, HEADER_STAMP_SECONDS);
  header.timestamp.nanos = Hdf5CoreGeneral::readAttributeFromHdf5<int32_t>(id, object, HEADER_STAMP_NANOS);
  header.frameId = Hdf5CoreGeneral::readFrameId(id, object, HEADER_FRAME_ID);
  header.sequence = Hdf5CoreGeneral::readAttributeFromHdf5<int32_t>(id, object, HEADER_SEQ);
}
}  // namespace seerep_hdf5_core
