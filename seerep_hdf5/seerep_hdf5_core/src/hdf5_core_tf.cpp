#include "seerep_hdf5_core/hdf5_core_tf.h"

namespace seerep_hdf5_core
{
Hdf5CoreTf::Hdf5CoreTf(std::shared_ptr<HighFive::File>& file,
                       std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx)
{
}

std::optional<std::vector<geometry_msgs::TransformStamped>>
Hdf5CoreTf::readTransformStamped(const std::string& id, const bool isStatic)
{
  const std::scoped_lock lock(*m_write_mtx);

  const std::string tfGroup = isStatic ? HDF5_GROUP_TF_STATIC : HDF5_GROUP_TF;
  const std::string hdf5GroupPath = tfGroup + "/" + id;
  const std::string hdf5DatasetTimePath = hdf5GroupPath + "/" + "time";
  const std::string hdf5DatasetTransPath = hdf5GroupPath + "/" + "translation";
  const std::string hdf5DatasetRotPath = hdf5GroupPath + "/" + "rotation";

  if (!m_file->exist(hdf5GroupPath) || !m_file->exist(hdf5DatasetTimePath) ||
      !m_file->exist(hdf5DatasetTransPath) ||
      !m_file->exist(hdf5DatasetRotPath))
  {
    return std::nullopt;
  }

  std::shared_ptr<HighFive::Group> group_ptr =
      std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));
  size_t size;
  group_ptr->getAttribute(SIZE).read(size);
  if (size == 0)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "tf data has size 0.";
    return std::nullopt;
  }

  // read frames
  std::string parentframe = readFrame("PARENT_FRAME", group_ptr);
  std::string childframe = readFrame("CHILD_FRAME", group_ptr);
  std::vector<std::vector<int64_t>> time = readTimestamps(hdf5DatasetTimePath);
  std::vector<std::vector<double>> trans =
      readTranslations(hdf5DatasetTransPath);
  std::vector<std::vector<double>> rot = readRotations(hdf5DatasetRotPath);

  return convertToTfs(size, parentframe, childframe, time, trans, rot);
}

std::optional<std::vector<std::string>>
Hdf5CoreTf::readTransformStampedFrames(const std::string& id, bool isStatic)
{
  const std::scoped_lock lock(*m_write_mtx);

  const std::string tfGroup = isStatic ? HDF5_GROUP_TF_STATIC : HDF5_GROUP_TF;
  const std::string hdf5GroupPath = tfGroup + "/" + id;

  if (!m_file->exist(hdf5GroupPath))
  {
    return std::nullopt;
  }

  std::shared_ptr<HighFive::Group> group_ptr =
      std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));

  const std::string parentframe =
      readFrame(seerep_hdf5_core::Hdf5CoreTf::CHILD_FRAME, group_ptr);
  const std::string childframe =
      readFrame(seerep_hdf5_core::Hdf5CoreTf::PARENT_FRAME, group_ptr);

  return std::vector<std::string>{ parentframe, childframe };
}

std::string Hdf5CoreTf::readFrame(
    const std::string& frameName,
    const std::shared_ptr<const HighFive::Group>& group_ptr) const
{
  std::string frame;
  group_ptr->getAttribute(frameName).read(frame);
  return frame;
}

void Hdf5CoreTf::writeTimestamp(const std::string& datagroupPath,
                                std::array<int64_t, 2> timestamp)
{
  std::shared_ptr<HighFive::DataSet> dataset;

  uint64_t size = 0;
  const std::string timestampDatasetPath = datagroupPath + "/time";

  if (!m_file->exist(timestampDatasetPath))
  {
    HighFive::DataSpace dataspace({ 1, 2 },
                                  { HighFive::DataSpace::UNLIMITED, 2 });
    HighFive::DataSetCreateProps createProperties;
    createProperties.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 2 }));
    dataset =
        std::make_shared<HighFive::DataSet>(m_file->createDataSet<int64_t>(
            timestampDatasetPath, dataspace, createProperties));
  }
  else
  {
    dataset = std::make_shared<HighFive::DataSet>(
        m_file->getDataSet(timestampDatasetPath));
    HighFive::Group group = m_file->getGroup(datagroupPath);
    group.getAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE).read(size);
    dataset->resize({ size + 1, 2 });
  }
  dataset->select({ size, 0 }, { 1, 2 }).write(timestamp);
  m_file->flush();
}

void Hdf5CoreTf::writeTranslation(const std::string& datagroupPath,
                                  std::array<double, 3> translation)
{
  std::shared_ptr<HighFive::DataSet> dataset;
  uint64_t size = 0;
  const std::string translationDatasetPath =
      datagroupPath + seerep_hdf5_core::Hdf5CoreTf::HDF5_DATASET_TRANSLATION;

  if (!m_file->exist(translationDatasetPath))
  {
    HighFive::DataSpace dataspace({ 1, 3 },
                                  { HighFive::DataSpace::UNLIMITED, 3 });
    HighFive::DataSetCreateProps createProperties;
    createProperties.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 3 }));
    dataset = std::make_shared<HighFive::DataSet>(m_file->createDataSet<double>(
        translationDatasetPath, dataspace, createProperties));
  }
  else
  {
    dataset = std::make_shared<HighFive::DataSet>(
        m_file->getDataSet(translationDatasetPath));
    HighFive::Group group = m_file->getGroup(datagroupPath);
    group.getAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE).read(size);
    dataset->resize({ size + 1, 3 });
  }
  dataset->select({ size, 0 }, { 1, 3 }).write(translation);
  m_file->flush();
}

void Hdf5CoreTf::writeRotation(const std::string& datagroupPath,
                               std::array<double, 4> rotation)
{
  std::shared_ptr<HighFive::DataSet> dataset;
  uint64_t size = 0;
  const std::string rotationDatasetPath =
      datagroupPath + seerep_hdf5_core::Hdf5CoreTf::HDF5_DATASET_ROTATION;

  if (!m_file->exist(rotationDatasetPath))
  {
    HighFive::DataSpace dataspace({ 1, 4 },
                                  { HighFive::DataSpace::UNLIMITED, 4 });
    HighFive::DataSetCreateProps createProperties;
    createProperties.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 4 }));
    dataset = std::make_shared<HighFive::DataSet>(m_file->createDataSet<double>(
        rotationDatasetPath, dataspace, createProperties));
  }
  else
  {
    dataset = std::make_shared<HighFive::DataSet>(
        m_file->getDataSet(rotationDatasetPath));
    HighFive::Group group = m_file->getGroup(datagroupPath);
    group.getAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE).read(size);
    dataset->resize({ size + 1, 4 });
  }
  dataset->select({ size, 0 }, { 1, 4 }).write(rotation);
  m_file->flush();
}

std::vector<std::vector<int64_t>>
Hdf5CoreTf::readTimestamps(const std::string& hdf5DatasetPath) const
{
  std::shared_ptr<HighFive::DataSet> data_set_time_ptr =
      std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetPath));
  std::vector<std::vector<int64_t>> time;
  data_set_time_ptr->read(time);

  return time;
}
std::vector<std::vector<double>>
Hdf5CoreTf::readTranslations(const std::string& hdf5DatasetTransPath) const
{
  std::shared_ptr<HighFive::DataSet> data_set_trans_ptr =
      std::make_shared<HighFive::DataSet>(
          m_file->getDataSet(hdf5DatasetTransPath));
  std::vector<std::vector<double>> trans;
  data_set_trans_ptr->read(trans);

  return trans;
}
std::vector<std::vector<double>>
Hdf5CoreTf::readRotations(const std::string& hdf5DatasetRotPath) const
{
  std::shared_ptr<HighFive::DataSet> data_set_rot_ptr =
      std::make_shared<HighFive::DataSet>(
          m_file->getDataSet(hdf5DatasetRotPath));
  std::vector<std::vector<double>> rot;
  data_set_rot_ptr->read(rot);

  return rot;
}

std::optional<std::vector<geometry_msgs::TransformStamped>>
Hdf5CoreTf::convertToTfs(const long unsigned int& size,
                         const std::string& parentframe,
                         const std::string& childframe,
                         const std::vector<std::vector<int64_t>>& timestamps,
                         const std::vector<std::vector<double>>& translations,
                         const std::vector<std::vector<double>>& rotations)
{
  // check if all have the right size
  if (timestamps.size() != size || translations.size() != size ||
      rotations.size() != size)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "sizes of time (" << timestamps.size() << "), translation ("
        << translations.size() << ") and rotation (" << rotations.size()
        << ") not matching. Size expected by value in metadata (" << size
        << ")";
    return std::nullopt;
  }

  std::vector<geometry_msgs::TransformStamped> tfs;
  for (long unsigned int i = 0; i < size; i++)
  {
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = parentframe;
    tf.child_frame_id = childframe;

    tf.header.stamp.sec = timestamps.at(i).at(0);
    tf.header.stamp.nsec = timestamps.at(i).at(1);

    tf.transform.translation.x = translations.at(i).at(0);
    tf.transform.translation.y = translations.at(i).at(1);
    tf.transform.translation.z = translations.at(i).at(2);

    tf.transform.rotation.x = rotations.at(i).at(0);
    tf.transform.rotation.y = rotations.at(i).at(1);
    tf.transform.rotation.z = rotations.at(i).at(2);
    tf.transform.rotation.w = rotations.at(i).at(3);

    tfs.push_back(tf);
  }

  return tfs;
}

}  // namespace seerep_hdf5_core
