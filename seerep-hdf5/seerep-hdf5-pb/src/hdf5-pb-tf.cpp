#include "seerep-hdf5-pb/hdf5-pb-tf.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_pb
{
Hdf5PbTf::Hdf5PbTf(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx), Hdf5PbGeneral(file, write_mtx), seerep_hdf5_core::Hdf5CoreTf(file, write_mtx)
{
}

void Hdf5PbTf::writeTransformStamped(const seerep::TransformStamped& tf)
{
  std::string datagroupPath = HDF5_GROUP_TF + "/" + tf.header().frame_id() + "_" + tf.child_frame_id();
  const std::scoped_lock lock(*m_write_mtx);

  std::shared_ptr<HighFive::Group> group;

  if (!m_file->exist(datagroupPath))
  {
    group = std::make_shared<HighFive::Group>(m_file->createGroup(datagroupPath));
    group->createAttribute<uint64_t>(seerep_hdf5_core::Hdf5CoreTf::SIZE, 0);
    writeAttributeToHdf5<std::string>(*group, "PARENT_FRAME", tf.header().frame_id());
    writeAttributeToHdf5<std::string>(*group, "CHILD_FRAME", tf.child_frame_id());
  }
  else
  {
    group = std::make_shared<HighFive::Group>(m_file->getGroup(datagroupPath));
  }

  writeTimestamp(datagroupPath, { tf.header().stamp().seconds(), tf.header().stamp().nanos() });
  writeTranslation(datagroupPath, { tf.transform().translation().x(), tf.transform().translation().y(),
                                    tf.transform().translation().z() });
  writeRotation(datagroupPath, { tf.transform().rotation().x(), tf.transform().rotation().y(),
                                 tf.transform().rotation().z(), tf.transform().rotation().w() });
  uint64_t size = readAttributeFromHdf5<uint64_t>(tf.header().uuid_msgs(), *group, seerep_hdf5_core::Hdf5CoreTf::SIZE);
  writeAttributeToHdf5<uint64_t>(*group, seerep_hdf5_core::Hdf5CoreTf::SIZE, size + 1);
}

std::optional<std::vector<seerep::TransformStamped>> Hdf5PbTf::readTransformStamped(const std::string& id)
{
  std::string hdf5GroupPath = HDF5_GROUP_TF + "/" + id;
  std::string hdf5DatasetTimePath = hdf5GroupPath + "/" + "time";
  std::string hdf5DatasetTransPath = hdf5GroupPath + "/" + "translation";
  std::string hdf5DatasetRotPath = hdf5GroupPath + "/" + "rotation";

  if (!m_file->exist(hdf5GroupPath) || !m_file->exist(hdf5DatasetTimePath) || !m_file->exist(hdf5DatasetTransPath) ||
      !m_file->exist(hdf5DatasetRotPath))
  {
    return std::nullopt;
  }

  std::cout << "loading " << hdf5GroupPath << std::endl;

  // read size
  std::shared_ptr<HighFive::Group> group_ptr = std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));
  long unsigned int size;
  group_ptr->getAttribute(SIZE).read(size);
  if (size == 0)
  {
    std::cout << "tf data has size 0." << std::endl;
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
    std::cout << "sizes of time (" << time.size() << "), translation (" << trans.size() << ") and rotation ("
              << rot.size() << ") not matching. Size expected by value in metadata (" << size << ")" << std::endl;
    return std::nullopt;
  }

  std::vector<seerep::TransformStamped> tfs;
  for (long unsigned int i = 0; i < size; i++)
  {
    seerep::TransformStamped tf;
    tf.mutable_header()->set_frame_id(parentframe);
    tf.set_child_frame_id(childframe);

    tf.mutable_header()->mutable_stamp()->set_seconds(time.at(i).at(0));
    tf.mutable_header()->mutable_stamp()->set_nanos(time.at(i).at(1));

    seerep::Vector3 translation;
    translation.set_x(trans.at(i).at(0));
    translation.set_y(trans.at(i).at(1));
    translation.set_z(trans.at(i).at(2));
    *tf.mutable_transform()->mutable_translation() = translation;

    seerep::Quaternion rotation;
    rotation.set_x(rot.at(i).at(0));
    rotation.set_y(rot.at(i).at(1));
    rotation.set_z(rot.at(i).at(2));
    rotation.set_w(rot.at(i).at(3));
    *tf.mutable_transform()->mutable_rotation() = rotation;

    tfs.push_back(tf);
  }
  return tfs;
}

std::optional<std::vector<std::string>> Hdf5PbTf::readTransformStampedFrames(const std::string& id)
{
  std::string hdf5GroupPath = HDF5_GROUP_TF + "/" + id;

  if (!m_file->exist(hdf5GroupPath))
  {
    return std::nullopt;
  }

  std::shared_ptr<HighFive::Group> group_ptr = std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));

  std::cout << "loading parent frame of " << hdf5GroupPath << std::endl;

  // read frames
  std::string parentframe;
  group_ptr->getAttribute("PARENT_FRAME").read(parentframe);
  std::string childframe;
  group_ptr->getAttribute("CHILD_FRAME").read(childframe);

  return std::vector<std::string>{ parentframe, childframe };
}

} /* namespace seerep_hdf5_pb */
