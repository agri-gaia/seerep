
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

template <class T>
std::shared_ptr<HighFive::DataSet> Hdf5CoreGeneral::getHdf5DataSet(const std::string& hdf5DataSetPath,
                                                                   HighFive::DataSpace& dataSpace)
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
    return std::make_shared<HighFive::DataSet>(m_file->createDataSet<T>(hdf5DataSetPath, dataSpace));
  }
}

}  // namespace seerep_hdf5_core
