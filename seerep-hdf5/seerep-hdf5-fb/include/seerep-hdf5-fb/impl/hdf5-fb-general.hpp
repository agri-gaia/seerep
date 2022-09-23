
namespace seerep_hdf5_fb
{
template <class T>
flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<seerep::fb::UnionMapEntry>>>
Hdf5FbGeneral::readAttributeMap(HighFive::AnnotateTraits<T>& object, flatbuffers::grpc::MessageBuilder& builder)
{
  std::vector<std::string> attributeList = object.listAttributeNames();

  std::vector<flatbuffers::Offset<seerep::fb::UnionMapEntry>> mapEntryVector;
  for (std::string attributeName : attributeList)
  {
    auto attribute = object.getAttribute(attributeName);

    if (attribute.getDataType().getClass() == HighFive::DataTypeClass::Integer)
    {
      int attributeValue;
      attribute.read(attributeValue);

      auto keyOffset = builder.CreateString(attributeName);

      seerep::fb::IntegerBuilder integerBuilder(builder);
      integerBuilder.add_data(attributeValue);
      auto integerOffset = integerBuilder.Finish();

      seerep::fb::UnionMapEntryBuilder unionMapEntryBuilder(builder);
      unionMapEntryBuilder.add_key(keyOffset);
      unionMapEntryBuilder.add_value_type(seerep::fb::Datatypes_Integer);
      unionMapEntryBuilder.add_value(integerOffset.Union());

      mapEntryVector.push_back(unionMapEntryBuilder.Finish());
    }
  }
  return builder.CreateVector(mapEntryVector);
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
flatbuffers::Offset<seerep::fb::Header> Hdf5FbGeneral::readHeaderAttributes(flatbuffers::grpc::MessageBuilder& builder,
                                                                            HighFive::AnnotateTraits<T>& object,
                                                                            std::string uuidMsg)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug) << "loading flatbuffers header attributes";

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
