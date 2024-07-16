
namespace seerep_hdf5_pb
{
template <class T>
void Hdf5PbGeneral::writeHeaderAttributes(HighFive::AnnotateTraits<T>& object,
                                          const seerep::pb::Header& header)
{
  writeAttributeToHdf5<int64_t>(
      object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS,
      header.stamp().seconds());
  writeAttributeToHdf5<int32_t>(
      object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS,
      header.stamp().nanos());
  writeFrameId(object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID,
               header.frame_id());
  writeAttributeToHdf5<uint32_t>(
      object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ, header.seq());
}

template <class T>
seerep::pb::Header
Hdf5PbGeneral::readHeaderAttributes(HighFive::AnnotateTraits<T>& object,
                                    const std::string& id)
{
  seerep::pb::Header header;

  header.mutable_stamp()->set_seconds(readAttributeFromHdf5<int64_t>(
      object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_SECONDS, id));
  header.mutable_stamp()->set_nanos(readAttributeFromHdf5<int32_t>(
      object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_STAMP_NANOS, id));
  header.set_frame_id(readFrameId(
      object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_FRAME_ID, id));
  header.set_seq(readAttributeFromHdf5<uint32_t>(
      object, seerep_hdf5_core::Hdf5CoreGeneral::HEADER_SEQ, id));
  header.set_uuid_project(
      std::filesystem::path(m_file->getName()).filename().stem());
  header.set_uuid_msgs(id);

  return header;
}

} /* namespace seerep_hdf5_pb */
