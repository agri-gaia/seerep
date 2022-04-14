
namespace seerep_hdf5_core
{
template <typename T, class C>
T Hdf5CoreGeneral::getAttribute(const std::string& id, const HighFive::AnnotateTraits<C>& object,
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

}  // namespace seerep_hdf5_core
