
namespace seerep_io_pb
{
template <typename T>
void IoPbGeneral::writeAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField,
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
T IoPbGeneral::getAttribute(const std::string& id, const std::shared_ptr<HighFive::DataSet> dataSetPtr,
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
void IoPbGeneral::writeHeaderAttributes(HighFive::AnnotateTraits<T>& object, const seerep::Header& header)
{
  if (!object.hasAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS))
    object.createAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS, header.stamp().seconds());
  else
    object.getAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS).write(header.stamp().seconds());

  if (!object.hasAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS))
    object.createAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS, header.stamp().nanos());
  else
    object.getAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS).write(header.stamp().nanos());

  if (!object.hasAttribute(seerep_io_core::IoCoreGeneral::HEADER_FRAME_ID))
    object.createAttribute(seerep_io_core::IoCoreGeneral::HEADER_FRAME_ID, header.frame_id());
  else
    object.getAttribute(seerep_io_core::IoCoreGeneral::HEADER_FRAME_ID).write(header.frame_id());

  if (!object.hasAttribute(seerep_io_core::IoCoreGeneral::HEADER_SEQ))
    object.createAttribute(seerep_io_core::IoCoreGeneral::HEADER_SEQ, header.seq());
  else
    object.getAttribute(seerep_io_core::IoCoreGeneral::HEADER_SEQ).write(header.seq());
}

template <class T>
seerep::Header IoPbGeneral::readHeaderAttributes(HighFive::AnnotateTraits<T>& object)
{
  seerep::Header header;

  int64_t seconds;
  int32_t nanos;
  uint32_t seq;

  object.getAttribute(seerep_io_core::IoCoreGeneral::HEADER_FRAME_ID).read(header.mutable_frame_id());

  object.getAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_SECONDS).read(seconds);
  object.getAttribute(seerep_io_core::IoCoreGeneral::HEADER_STAMP_NANOS).read(nanos);
  object.getAttribute(seerep_io_core::IoCoreGeneral::HEADER_SEQ).read(seq);

  header.set_seq(seq);
  header.mutable_stamp()->set_seconds(seconds);
  header.mutable_stamp()->set_nanos(nanos);

  return header;
}

} /* namespace seerep_io_pb */
