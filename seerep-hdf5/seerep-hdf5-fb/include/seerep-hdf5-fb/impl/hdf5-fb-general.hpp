
namespace seerep_hdf5_fb
{
template <typename T>
void Hdf5FbGeneral::writeAttribute(const std::shared_ptr<HighFive::DataSet> dataSetPtr, std::string attributeField,
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
T Hdf5FbGeneral::getAttribute(const std::string& id, const std::shared_ptr<HighFive::DataSet> dataSetPtr,
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
void Hdf5FbGeneral::writeHeaderAttributes(HighFive::AnnotateTraits<T>& object, const seerep::fb::Header& header)
{
  if (!object.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS))
    object.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS, header.stamp()->seconds());
  else
    object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS).write(header.stamp()->seconds());

  if (!object.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS))
    object.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS, header.stamp()->nanos());
  else
    object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS).write(header.stamp()->nanos());

  if (!object.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID))
    object.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID, header.frame_id()->str());
  else
    object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID).write(header.frame_id()->str());

  if (!object.hasAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ))
    object.createAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ, header.seq());
  else
    object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ).write(header.seq());
}

template <class T>
flatbuffers::Offset<seerep::fb::Header> Hdf5FbGeneral::readHeaderAttributes(HighFive::AnnotateTraits<T>& object,
                                                                            std::string uuidMsg,
                                                                            flatbuffers::grpc::MessageBuilder& builder)
{
  int64_t seconds;
  int32_t nanos;
  uint32_t seq;
  std::string frameId;

  std::string uuidProject = std::filesystem::path(m_file->getName()).filename().stem();

  object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID).read(frameId);

  object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS).read(seconds);
  object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS).read(nanos);
  object.getAttribute(seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ).read(seq);

  auto timestamp = seerep::fb::CreateTimestamp(builder, seconds, nanos);

  return seerep::fb::CreateHeader(builder, seq, timestamp, builder.CreateString(frameId),
                                  builder.CreateString(uuidProject), builder.CreateString(uuidMsg));
}

} /* namespace seerep_hdf5_fb */
