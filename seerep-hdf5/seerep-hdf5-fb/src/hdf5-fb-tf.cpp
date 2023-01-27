#include "seerep-hdf5-fb/hdf5-fb-tf.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_fb
{
Hdf5FbTf::Hdf5FbTf(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx), Hdf5FbGeneral(file, write_mtx), seerep_hdf5_core::Hdf5CoreTf(file, write_mtx)
{
}

void Hdf5FbTf::writeTransformStamped(const seerep::fb::TransformStamped& tf)
{
  const std::scoped_lock lock(*m_write_mtx);

  const std::string datagroupPath = seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF + "/" + tf.header()->frame_id()->str() +
                                    "_" + tf.child_frame_id()->str();
  std::shared_ptr<HighFive::Group> group;

  if (!m_file->exist(datagroupPath))
  {
    group = std::make_shared<HighFive::Group>(m_file->createGroup(datagroupPath));
    writeAttributeToHdf5<std::string>(*group, "PARENT_FRAME", tf.header()->frame_id()->str());
    writeAttributeToHdf5<std::string>(*group, "CHILD_FRAME", tf.child_frame_id()->str());
  }
  else
  {
    group = std::make_shared<HighFive::Group>(m_file->getGroup(datagroupPath));
  }

  writeTimestamp(datagroupPath, { tf.header()->stamp()->seconds(), tf.header()->stamp()->nanos() });
  writeTranslation(datagroupPath, { tf.transform()->translation()->x(), tf.transform()->translation()->y(),
                                    tf.transform()->translation()->z() });
  writeRotation(datagroupPath, { tf.transform()->rotation()->x(), tf.transform()->rotation()->y(),
                                 tf.transform()->rotation()->z(), tf.transform()->rotation()->w() });
  uint64_t size =
      readAttributeFromHdf5<uint64_t>(tf.header()->uuid_msgs()->str(), *group, seerep_hdf5_core::Hdf5CoreTf::SIZE);
  writeAttributeToHdf5<uint64_t>(*group, seerep_hdf5_core::Hdf5CoreTf::SIZE, size + 1);
  m_file->flush();
}

std::optional<std::vector<flatbuffers::Offset<seerep::fb::TransformStamped>>>
Hdf5FbTf::readTransformStamped(const std::string& id)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5GroupPath = seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF + "/" + id;
  std::string hdf5DatasetTimePath = hdf5GroupPath + "/" + "time";
  std::string hdf5DatasetTransPath = hdf5GroupPath + "/" + "translation";
  std::string hdf5DatasetRotPath = hdf5GroupPath + "/" + "rotation";

  if (!m_file->exist(hdf5GroupPath) || !m_file->exist(hdf5DatasetTimePath) || !m_file->exist(hdf5DatasetTransPath) ||
      !m_file->exist(hdf5DatasetRotPath))
  {
    return std::nullopt;
  }

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "loading " << hdf5GroupPath;

  // read size
  std::shared_ptr<HighFive::Group> group_ptr = std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));
  long unsigned int size;
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE).read(size);
  if (size == 0)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning) << "tf data has size 0.";
    return std::nullopt;
  }

  // read frames
  std::string parentframe;
  group_ptr->getAttribute("PARENT_FRAME").read(parentframe);
  std::string childframe;
  group_ptr->getAttribute("CHILD_FRAME").read(childframe);

  // read time
  std::shared_ptr<HighFive::DataSet> data_set_time_ptr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetTimePath));
  std::vector<std::vector<int64_t>> time;
  data_set_time_ptr->read(time);

  // read translation
  std::shared_ptr<HighFive::DataSet> data_set_trans_ptr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetTransPath));
  std::vector<std::vector<double>> trans;
  data_set_trans_ptr->read(trans);

  // read rotation
  std::shared_ptr<HighFive::DataSet> data_set_rot_ptr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRotPath));
  std::vector<std::vector<double>> rot;
  data_set_rot_ptr->read(rot);

  // check if all have the right size
  if (time.size() != size || trans.size() != size || rot.size() != size)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "sizes of time (" << time.size() << "), translation (" << trans.size() << ") and rotation (" << rot.size()
        << ") not matching. Size expected by value in metadata (" << size << ")";
    return std::nullopt;
  }

  std::vector<flatbuffers::Offset<seerep::fb::TransformStamped>> tfs;
  flatbuffers::FlatBufferBuilder builder;
  seerep::fb::TransformStampedBuilder tfBuilder(builder);
  for (long unsigned int i = 0; i < size; i++)
  {
    tfBuilder.add_child_frame_id(builder.CreateString(childframe));
    tfBuilder.add_header(seerep::fb::CreateHeader(
        builder, 0, seerep::fb::CreateTimestamp(builder, time.at(i).at(0), time.at(i).at(1)),
        builder.CreateString(parentframe), builder.CreateString(""), builder.CreateString("")));

    tfBuilder.add_transform(seerep::fb::CreateTransform(
        builder, seerep::fb::CreateVector3(builder, trans.at(i).at(0), trans.at(i).at(1), trans.at(i).at(2)),
        seerep::fb::CreateQuaternion(builder, rot.at(i).at(0), rot.at(i).at(1), rot.at(i).at(2))));

    tfs.push_back(tfBuilder.Finish());
  }
  return tfs;
}

std::optional<std::vector<std::string>> Hdf5FbTf::readTransformStampedFrames(const std::string& id)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5GroupPath = seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF + "/" + id;

  if (!m_file->exist(hdf5GroupPath))
  {
    return std::nullopt;
  }

  std::shared_ptr<HighFive::Group> group_ptr = std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "loading parent frame of " << hdf5GroupPath;

  // read frames
  std::string parentframe;
  group_ptr->getAttribute("PARENT_FRAME").read(parentframe);
  std::string childframe;
  group_ptr->getAttribute("CHILD_FRAME").read(childframe);

  return std::vector<std::string>{ parentframe, childframe };
}

}  // namespace seerep_hdf5_fb
