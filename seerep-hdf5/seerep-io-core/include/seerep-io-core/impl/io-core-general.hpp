
namespace seerep_io_core
{
template <typename T>
T IoCoreGeneral::getAttribute(const std::string& id, const std::shared_ptr<HighFive::DataSet> dataSetPtr,
                              std::string attributeField)
{
  T attributeValue;
  if (dataSetPtr->hasAttribute(attributeField))
  {
    dataSetPtr->getAttribute(attributeField).read(attributeValue);
  }
  else
  {
    throw std::invalid_argument("id " + id + " has no attribute " + attributeField);
  }
  return attributeValue;
}

}  // namespace seerep_io_core
