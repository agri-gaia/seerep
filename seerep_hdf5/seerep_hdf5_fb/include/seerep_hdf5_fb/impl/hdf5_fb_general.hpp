
namespace seerep_hdf5_fb
{
template <class T>
flatbuffers::Offset<AttributeMapsFb>
Hdf5FbGeneral::readAttributeMap(HighFive::AnnotateTraits<T>& object,
                                flatbuffers::grpc::MessageBuilder& builder)
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

      mapEntryVector.push_back(seerep::fb::CreateUnionMapEntryDirect(
          builder, attributeName.c_str(), seerep::fb::Datatypes_Integer,
          seerep::fb::CreateInteger(builder, attributeValue).Union()));
    }
    else if (attribute.getDataType().getClass() ==
             HighFive::DataTypeClass::Float)
    {
      double attributeValue;
      attribute.read(attributeValue);

      mapEntryVector.push_back(seerep::fb::CreateUnionMapEntryDirect(
          builder, attributeName.c_str(), seerep::fb::Datatypes_Double,
          seerep::fb::CreateDouble(builder, attributeValue).Union()));
    }
    else if (attribute.getDataType().getClass() ==
             HighFive::DataTypeClass::String)
    {
      std::string attributeValue;
      attribute.read(attributeValue);

      mapEntryVector.push_back(seerep::fb::CreateUnionMapEntryDirect(
          builder, attributeName.c_str(), seerep::fb::Datatypes_String,
          seerep::fb::CreateString(builder, builder.CreateString(attributeValue))
              .Union()));
    }
    else
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
          << "read of data type of attribute not implemented.";
    }
  }
  return builder.CreateVector(mapEntryVector);
}

template <class T>
void Hdf5FbGeneral::writeHeaderAttributes(HighFive::AnnotateTraits<T>& object,
                                          const seerep::fb::Header* header)
{
  if (header)
  {
    writeAttributeToHdf5<int64_t>(
        object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS,
        header->stamp()->seconds());
    writeAttributeToHdf5<int32_t>(
        object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS,
        header->stamp()->nanos());
    writeFrameId(object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID,
                 header->frame_id()->str());
    writeAttributeToHdf5<int32_t>(
        object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ, header->seq());
  }
}

template <class T>
flatbuffers::Offset<seerep::fb::Header>
Hdf5FbGeneral::readHeaderAttributes(flatbuffers::grpc::MessageBuilder& builder,
                                    HighFive::AnnotateTraits<T>& object,
                                    std::string uuidMsg)
{
  std::string uuidProject =
      std::filesystem::path(m_file->getName()).filename().stem();

  int64_t seconds = readAttributeFromHdf5<int64_t>(
      object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS, uuidMsg);
  int32_t nanos = readAttributeFromHdf5<int32_t>(
      object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS, uuidMsg);
  std::string frameId = readFrameId(
      object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID, uuidMsg);

  uint32_t seq = readAttributeFromHdf5<uint32_t>(
      object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ, uuidMsg);

  auto timestamp = seerep::fb::CreateTimestamp(builder, seconds, nanos);

  return seerep::fb::CreateHeader(builder, seq, timestamp,
                                  builder.CreateString(frameId),
                                  builder.CreateString(uuidProject),
                                  builder.CreateString(uuidMsg));
}

} /* namespace seerep_hdf5_fb */
