namespace seerep_hdf5_fb
{

template <typename T>
void Hdf5FbPointCloud::writePointFieldAttributes(
    HighFive::AnnotateTraits<T>& object,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::PointField>>*
        pointFields)
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::debug)
      << "writing point field attributes to hdf5";

  if (pointFields)
  {
    std::vector<std::string> names;
    std::vector<uint32_t> offsets, counts;
    std::vector<uint8_t> datatypes;

    for (auto pointField : *pointFields)
    {
      names.push_back(pointField->name()->str());
      offsets.push_back(pointField->offset());
      datatypes.push_back(static_cast<uint8_t>(pointField->datatype()));
      counts.push_back(pointField->count());
    }

    writeAttributeToHdf5<std::vector<std::string>>(
        object, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_NAME, names);
    writeAttributeToHdf5<std::vector<uint32_t>>(
        object, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_OFFSET, offsets);
    writeAttributeToHdf5<std::vector<uint8_t>>(
        object, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_DATATYPE, datatypes);
    writeAttributeToHdf5<std::vector<uint32_t>>(
        object, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_COUNT, counts);
  }
}

} /* namespace seerep_hdf5_fb */
