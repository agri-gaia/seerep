#include <filesystem>

namespace seerep_hdf5_core
{
template <typename T, class C>
T Hdf5CoreGeneral::readAttributeFromHdf5(const HighFive::AnnotateTraits<C>& object, const std::string& attribute_name,
                                         const std::string& path)
{
  T attribute_value;
  if (object.hasAttribute(attribute_name))
  {
    object.getAttribute(attribute_name).read(attribute_value);
  }
  else
  {
    throw std::invalid_argument("Path: " + path + " has no attribute:" + attribute_name);
  }
  return attribute_value;
}

template <typename T, class C>
void Hdf5CoreGeneral::writeAttributeToHdf5(HighFive::AnnotateTraits<C>& object, const std::string& attribute_name,
                                           T attribute_val)
{
  if (object.hasAttribute(attribute_name))
  {
    object.getAttribute(attribute_name).write(attribute_val);
  }
  else
  {
    object.createAttribute(attribute_name, attribute_val);
  }
}

template <class C>
std::string Hdf5CoreGeneral::readFrameId(const HighFive::AnnotateTraits<C>& object, const std::string& frame_field,
                                         const std::string& path)
{
  return tf2_frame_id(readAttributeFromHdf5<std::string>(object, frame_field, path));
}

template <class C>
void Hdf5CoreGeneral::writeFrameId(HighFive::AnnotateTraits<C>& object, const std::string& frame_field,
                                   const std::string& frame_id)
{
  writeAttributeToHdf5<std::string>(object, frame_field, tf2_frame_id(frame_id));
}

template <class T>
std::shared_ptr<HighFive::DataSet> Hdf5CoreGeneral::getHdf5DataSet(const std::string& hdf5DataSetPath,
                                                                   const HighFive::DataSpace& dataSpace)
{
  return getHdf5DataSet<T>(hdf5DataSetPath, dataSpace, HighFive::DataSetCreateProps());
}

template <class T>
std::shared_ptr<HighFive::DataSet> Hdf5CoreGeneral::getHdf5DataSet(const std::string& dataset_path,
                                                                   const HighFive::DataSpace& dataspace,
                                                                   const HighFive::DataSetCreateProps& properties)
{
  if (exists(dataset_path))
  {
    return std::make_shared<HighFive::DataSet>(m_file->getDataSet(dataset_path));
  }
  return std::make_shared<HighFive::DataSet>(m_file->createDataSet<T>(dataset_path, dataspace, properties));
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

  header.timestamp.seconds = Hdf5CoreGeneral::readAttributeFromHdf5<int32_t>(object, HEADER_STAMP_SECONDS, id);
  header.timestamp.nanos = Hdf5CoreGeneral::readAttributeFromHdf5<int32_t>(object, HEADER_STAMP_NANOS, id);
  header.frameId = Hdf5CoreGeneral::readFrameId(object, HEADER_FRAME_ID, id);
  header.sequence = Hdf5CoreGeneral::readAttributeFromHdf5<int32_t>(object, HEADER_SEQ, id);
}

template <class T>
T Hdf5CoreGeneral::readDataset(const std::string& path)
{
  checkExists(path);
  HighFive::DataSet datasetInstances = m_file->getDataSet(path);
  T data;
  datasetInstances.read(data);
  return data;
}

template <typename T0, typename... TN>
bool Hdf5CoreGeneral::hasEqualSize(const std::vector<T0>& first, const std::vector<TN>&... rest) const
{
  return ((first.size() == rest.size()) && ...);
}

}  // namespace seerep_hdf5_core
