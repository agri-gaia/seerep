#include "seerep_hdf5_pb/hdf5_pb_tf.h"

#include <highfive/H5DataSet.hpp>

#include "seerep_hdf5_core/hdf5_core_tf.h"

namespace seerep_hdf5_pb
{
Hdf5PbTf::Hdf5PbTf(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx), Hdf5PbGeneral(file, write_mtx)
{
}

void Hdf5PbTf::writeTransformStamped(const seerep::pb::TransformStamped& tf)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5_group_tf;
  if (tf.is_static())
  {
    hdf5_group_tf = seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF_STATIC;
  }
  else
  {
    hdf5_group_tf = seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF;
  }

  std::string hdf5DatasetPath = hdf5_group_tf + "/" + tf.header().frame_id() + "_" + tf.child_frame_id();
  std::string hdf5DatasetTimePath = hdf5DatasetPath + "/" + "time";
  std::string hdf5DatasetTransPath = hdf5DatasetPath + "/" + "translation";
  std::string hdf5DatasetRotPath = hdf5DatasetPath + "/" + "rotation";

  std::shared_ptr<HighFive::DataSet> data_set_time_ptr, data_set_trans_ptr, data_set_rot_ptr;
  uint64_t size = 0;

  if (!m_file->exist(hdf5DatasetPath))
  {
    std::cout << "data id " << hdf5DatasetPath << " does not exist! Creat new dataset in hdf5" << std::endl;
    HighFive::Group group = m_file->createGroup(hdf5DatasetPath);
    group.createAttribute(seerep_hdf5_core::Hdf5CoreTf::CHILD_FRAME, tf.child_frame_id());
    group.createAttribute(seerep_hdf5_core::Hdf5CoreTf::PARENT_FRAME, tf.header().frame_id());

    // TIME
    // Create a dataspace with initial shape and max shape
    HighFive::DataSpace data_space_time({ 1, 2 }, { HighFive::DataSpace::UNLIMITED, 2 });
    // Use chunking
    HighFive::DataSetCreateProps props_time;
    props_time.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 2 }));
    data_set_time_ptr = std::make_shared<HighFive::DataSet>(
        m_file->createDataSet<int64_t>(hdf5DatasetTimePath, data_space_time, props_time));

    // TRANSLATION
    // Create a dataspace with initial shape and max shape
    HighFive::DataSpace data_space_trans({ 1, 3 }, { HighFive::DataSpace::UNLIMITED, 3 });
    // Use chunking
    HighFive::DataSetCreateProps props_trans;
    props_trans.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 3 }));
    data_set_trans_ptr = std::make_shared<HighFive::DataSet>(
        m_file->createDataSet<double>(hdf5DatasetTransPath, data_space_trans, props_trans));

    // ROTATION
    // Create a dataspace with initial shape and max shape
    HighFive::DataSpace data_space_rot({ 1, 4 }, { HighFive::DataSpace::UNLIMITED, 4 });
    // Use chunking
    HighFive::DataSetCreateProps props_rot;
    props_rot.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 4 }));
    data_set_rot_ptr = std::make_shared<HighFive::DataSet>(
        m_file->createDataSet<double>(hdf5DatasetRotPath, data_space_rot, props_rot));
  }
  else
  {
    std::cout << "data id " << hdf5DatasetPath << " already exists!" << std::endl;
    data_set_time_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetTimePath));
    data_set_trans_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetTransPath));
    data_set_rot_ptr = std::make_shared<HighFive::DataSet>(m_file->getDataSet(hdf5DatasetRotPath));

    HighFive::Group group = m_file->getGroup(hdf5DatasetPath);
    group.getAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE).read(size);

    // Resize the dataset to a larger size
    data_set_time_ptr->resize({ size + 1, 2 });
    data_set_trans_ptr->resize({ size + 1, 3 });
    data_set_rot_ptr->resize({ size + 1, 4 });
  }

  // write time
  std::vector<int64_t> time;
  time.push_back(tf.header().stamp().seconds());
  time.push_back(tf.header().stamp().nanos());
  data_set_time_ptr->select({ size, 0 }, { 1, 2 }).write(time);

  // write translation
  std::vector<double> trans;
  trans.push_back(tf.transform().translation().x());
  trans.push_back(tf.transform().translation().y());
  trans.push_back(tf.transform().translation().z());
  data_set_trans_ptr->select({ size, 0 }, { 1, 3 }).write(trans);

  // write rotation
  std::vector<double> rot;
  rot.push_back(tf.transform().rotation().x());
  rot.push_back(tf.transform().rotation().y());
  rot.push_back(tf.transform().rotation().z());
  rot.push_back(tf.transform().rotation().w());
  data_set_rot_ptr->select({ size, 0 }, { 1, 4 }).write(rot);

  // write the size as group attribute
  HighFive::Group group = m_file->getGroup(hdf5DatasetPath);
  if (!group.hasAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE))
  {
    group.createAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE, ++size);
  }
  else
  {
    group.getAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE).write(++size);
  }

  m_file->flush();
}

std::optional<std::vector<seerep::pb::TransformStamped>> Hdf5PbTf::readTransformStamped(const std::string& id,
                                                                                        const bool isStatic)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5_group_tf;
  if (isStatic)
  {
    hdf5_group_tf = seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF_STATIC;
  }
  else
  {
    hdf5_group_tf = seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF;
  }

  std::string hdf5GroupPath = hdf5_group_tf + "/" + id;
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
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE).read(size);
  if (size == 0)
  {
    std::cout << "tf data has size 0." << std::endl;
    return std::nullopt;
  }

  // read frames
  std::string parentframe;
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::PARENT_FRAME).read(parentframe);
  std::string childframe;
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::CHILD_FRAME).read(childframe);

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

  std::vector<seerep::pb::TransformStamped> tfs;
  for (long unsigned int i = 0; i < size; i++)
  {
    seerep::pb::TransformStamped tf;
    tf.mutable_header()->set_frame_id(parentframe);
    tf.set_child_frame_id(childframe);

    tf.mutable_header()->mutable_stamp()->set_seconds(time.at(i).at(0));
    tf.mutable_header()->mutable_stamp()->set_nanos(time.at(i).at(1));

    seerep::pb::Vector3 translation;
    translation.set_x(trans.at(i).at(0));
    translation.set_y(trans.at(i).at(1));
    translation.set_z(trans.at(i).at(2));
    *tf.mutable_transform()->mutable_translation() = translation;

    seerep::pb::Quaternion rotation;
    rotation.set_x(rot.at(i).at(0));
    rotation.set_y(rot.at(i).at(1));
    rotation.set_z(rot.at(i).at(2));
    rotation.set_w(rot.at(i).at(3));
    *tf.mutable_transform()->mutable_rotation() = rotation;

    tfs.push_back(tf);
  }
  return tfs;
}

std::optional<std::vector<std::string>> Hdf5PbTf::readTransformStampedFrames(const std::string& id, const bool isStatic)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string hdf5_group_tf;
  if (isStatic)
  {
    hdf5_group_tf = seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF_STATIC;
  }
  else
  {
    hdf5_group_tf = seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF;
  }

  std::string hdf5GroupPath = hdf5_group_tf + "/" + id;

  if (!m_file->exist(hdf5GroupPath))
  {
    return std::nullopt;
  }

  std::shared_ptr<HighFive::Group> group_ptr = std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));

  std::cout << "loading parent frame of " << hdf5GroupPath << std::endl;

  // read frames
  std::string parentframe;
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::PARENT_FRAME).read(parentframe);
  std::string childframe;
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::CHILD_FRAME).read(childframe);

  return std::vector<std::string>{ parentframe, childframe };
}

} /* namespace seerep_hdf5_pb */
