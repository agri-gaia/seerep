
namespace seerep_hdf5_pb
{
template <typename T>
void Hdf5PbGeneral::writeAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField,
                                   T value)
{
  if (!dataSetPtr->hasAttribute(attributeField))
  {
    dataSetPtr->createAttribute(attributeField, value);
  }
  else
  {
    dataSetPtr->getAttribute(attributeField).write(value);
  }
  m_file->flush();
}

template <typename T>
T Hdf5PbGeneral::getAttribute(const std::string& id, const std::shared_ptr<HighFive::DataSet> dataSetPtr,
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

template <class T>
void Hdf5PbGeneral::writeHeaderAttributes(HighFive::AnnotateTraits<T>& object, const seerep::Header& header)
{
  if (!object.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS))
    object.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS, header.stamp().seconds());
  else
    object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS).write(header.stamp().seconds());

  if (!object.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS))
    object.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS, header.stamp().nanos());
  else
    object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS).write(header.stamp().nanos());

  if (!object.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID))
    object.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID, header.frame_id());
  else
    object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID).write(header.frame_id());

  if (!object.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ))
    object.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ, header.seq());
  else
    object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ).write(header.seq());
}

template <class T>
seerep::Header Hdf5PbGeneral::readHeaderAttributes(HighFive::AnnotateTraits<T>& object, const std::string& id)
{
  seerep::Header header;

  int64_t seconds;
  int32_t nanos;
  uint32_t seq;

  std::string uuidProject = std::filesystem::path(m_file->getName()).filename().stem();

  object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID).read(*header.mutable_frame_id());

  object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS).read(seconds);
  object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS).read(nanos);
  object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ).read(seq);

  header.set_seq(seq);
  header.mutable_stamp()->set_seconds(seconds);
  header.mutable_stamp()->set_nanos(nanos);
  header.set_uuid_project(uuidProject);
  header.set_uuid_msgs(id);

  return header;
}

} /* namespace seerep_hdf5_pb */
