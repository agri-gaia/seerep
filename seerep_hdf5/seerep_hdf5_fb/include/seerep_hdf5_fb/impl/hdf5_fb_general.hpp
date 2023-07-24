
namespace seerep_hdf5_fb
{
template <class T>
flatbuffers::Offset<AttributeMapsFb> Hdf5FbGeneral::readAttributeMap(HighFive::AnnotateTraits<T>& object,
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
    else if (attribute.getDataType().getClass() == HighFive::DataTypeClass::Float)
    {
      double attributeValue;
      attribute.read(attributeValue);

      auto keyOffset = builder.CreateString(attributeName);

      seerep::fb::DoubleBuilder doubleBuilder(builder);
      doubleBuilder.add_data(attributeValue);
      auto doubleOffset = doubleBuilder.Finish();

      seerep::fb::UnionMapEntryBuilder unionMapEntryBuilder(builder);
      unionMapEntryBuilder.add_key(keyOffset);
      unionMapEntryBuilder.add_value_type(seerep::fb::Datatypes_Double);
      unionMapEntryBuilder.add_value(doubleOffset.Union());

      mapEntryVector.push_back(unionMapEntryBuilder.Finish());
    }
    else if (attribute.getDataType().getClass() == HighFive::DataTypeClass::String)
    {
      std::string attributeValue;
      attribute.read(attributeValue);
      auto valueOffset = builder.CreateString(attributeValue);

      auto keyOffset = builder.CreateString(attributeName);

      seerep::fb::StringBuilder stringBuilder(builder);
      stringBuilder.add_data(valueOffset);
      auto stringOffset = stringBuilder.Finish();

      seerep::fb::UnionMapEntryBuilder unionMapEntryBuilder(builder);
      unionMapEntryBuilder.add_key(keyOffset);
      unionMapEntryBuilder.add_value_type(seerep::fb::Datatypes_String);
      unionMapEntryBuilder.add_value(stringOffset.Union());

      mapEntryVector.push_back(unionMapEntryBuilder.Finish());
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
void Hdf5FbGeneral::writeHeaderAttributes(HighFive::AnnotateTraits<T>& object, const seerep::fb::Header* header)
{
  if (header)
  {
    writeAttributeToHdf5<int64_t>(object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS,
                                  header->stamp()->seconds());
    writeAttributeToHdf5<int32_t>(object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS,
                                  header->stamp()->nanos());
    writeAttributeToHdf5<std::string>(object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID,
                                      header->frame_id()->str());
    writeAttributeToHdf5<int32_t>(object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ, header->seq());
  }
}

template <class T>
flatbuffers::Offset<seerep::fb::Header> Hdf5FbGeneral::readHeaderAttributes(flatbuffers::grpc::MessageBuilder& builder,
                                                                            HighFive::AnnotateTraits<T>& object,
                                                                            std::string uuidMsg)
{
  std::string uuidProject = std::filesystem::path(m_file->getName()).filename().stem();

  int64_t seconds =
      readAttributeFromHdf5<int64_t>(uuidMsg, object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS);
  int32_t nanos =
      readAttributeFromHdf5<int32_t>(uuidMsg, object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS);
  std::string frameId =
      readAttributeFromHdf5<std::string>(uuidMsg, object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID);
  uint32_t seq = readAttributeFromHdf5<uint32_t>(uuidMsg, object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ);

  auto timestamp = seerep::fb::CreateTimestamp(builder, seconds, nanos);

  return seerep::fb::CreateHeader(builder, seq, timestamp, builder.CreateString(frameId),
                                  builder.CreateString(uuidProject), builder.CreateString(uuidMsg));
}

} /* namespace seerep_hdf5_fb */
