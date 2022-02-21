
namespace seerep_hdf5
{
template <typename T>
void GeneralIO::writeAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField, T value)
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
T GeneralIO::getAttribute(const std::string& id, const std::shared_ptr<HighFive::DataSet> dataSetPtr,
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
void GeneralIO::writeHeaderAttributes(HighFive::AnnotateTraits<T>& object, const seerep::Header& header)
{
  if (!object.hasAttribute(HEADER_STAMP_SECONDS))
    object.createAttribute(HEADER_STAMP_SECONDS, header.stamp().seconds());
  else
    object.getAttribute(HEADER_STAMP_SECONDS).write(header.stamp().seconds());

  if (!object.hasAttribute(HEADER_STAMP_NANOS))
    object.createAttribute(HEADER_STAMP_NANOS, header.stamp().nanos());
  else
    object.getAttribute(HEADER_STAMP_NANOS).write(header.stamp().nanos());

  if (!object.hasAttribute(HEADER_FRAME_ID))
    object.createAttribute(HEADER_FRAME_ID, header.frame_id());
  else
    object.getAttribute(HEADER_FRAME_ID).write(header.frame_id());

  if (!object.hasAttribute(HEADER_SEQ))
    object.createAttribute(HEADER_SEQ, header.seq());
  else
    object.getAttribute(HEADER_SEQ).write(header.seq());
}

template <class T>
seerep::Header GeneralIO::readHeaderAttributes(HighFive::AnnotateTraits<T>& object)
{
  seerep::Header header;

  int64_t seconds;
  int32_t nanos;
  uint32_t seq;

  object.getAttribute(HEADER_FRAME_ID).read(header.mutable_frame_id());

  object.getAttribute(HEADER_STAMP_SECONDS).read(seconds);
  object.getAttribute(HEADER_STAMP_NANOS).read(nanos);
  object.getAttribute(HEADER_SEQ).read(seq);

  header.set_seq(seq);
  header.mutable_stamp()->set_seconds(seconds);
  header.mutable_stamp()->set_nanos(nanos);

  return header;
}

} /* namespace seerep_hdf5 */
