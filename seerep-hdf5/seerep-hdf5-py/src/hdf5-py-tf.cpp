#include "seerep-hdf5-py/hdf5-py-tf.h"

#include <highfive/H5DataSet.hpp>

#include "seerep-hdf5-py/hdf5-py.h"

namespace seerep_hdf5_py
{

Hdf5PyTf::Hdf5PyTf(Hdf5FileWrapper& hdf5_file)
  : Hdf5CoreGeneral(hdf5_file.getFile(), hdf5_file.getMutex()), Hdf5PyGeneral(hdf5_file)
{
}

void Hdf5PyTf::writeTransformStamped(const TfTransform& tf)
{
  std::string hdf5_group_path = seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF + "/" + tf.frame_id;
  std::string hdf5_dataset_time_path = hdf5_group_path + "/" + "time";
  std::string hdf5_dataset_trans_path = hdf5_group_path + "/" + "translation";
  std::string hdf5_dataset_rot_path = hdf5_group_path + "/" + "rotation";

  std::shared_ptr<HighFive::Group> tf_group_ptr = getHdf5Group(hdf5_group_path);
  if (tf_group_ptr == nullptr)
  {
    throw std::invalid_argument("unable to obtain tf hdf5 group for frame " + tf.frame_id);
  }

  HighFive::DataSpace data_space_time({ 1, 2 }, { HighFive::DataSpace::UNLIMITED, 2 });
  HighFive::DataSetCreateProps props_time;
  props_time.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 2 }));
  std::shared_ptr<HighFive::DataSet> data_set_time_ptr =
      getHdf5DataSet<int64_t>(hdf5_dataset_time_path, data_space_time, props_time);

  HighFive::DataSpace data_space_trans({ 1, 3 }, { HighFive::DataSpace::UNLIMITED, 3 });
  HighFive::DataSetCreateProps props_trans;
  props_trans.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 3 }));
  std::shared_ptr<HighFive::DataSet> data_set_trans_ptr =
      getHdf5DataSet<double>(hdf5_dataset_trans_path, data_space_trans, props_trans);

  HighFive::DataSpace data_space_rot({ 1, 4 }, { HighFive::DataSpace::UNLIMITED, 4 });
  HighFive::DataSetCreateProps props_rot;
  props_rot.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 4 }));
  std::shared_ptr<HighFive::DataSet> data_set_rot_ptr =
      getHdf5DataSet<double>(hdf5_dataset_rot_path, data_space_rot, props_rot);

  if (data_set_time_ptr == nullptr || data_set_trans_ptr == nullptr || data_set_rot_ptr == nullptr)
  {
    throw std::invalid_argument("unable to obtain the required tf hdf5 datasets for frame " + tf.frame_id);
  }

  uint64_t size = 1;
  if (tf_group_ptr->hasAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE))
  {
    tf_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE).read(size);

    size++;

    data_set_time_ptr->resize({ size, 2 });
    data_set_trans_ptr->resize({ size, 3 });
    data_set_rot_ptr->resize({ size, 4 });

    tf_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE).write(size);
  }
  else
  {
    tf_group_ptr->createAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE, size);
  }

  // write frames
  writeAttributeToHdf5(*tf_group_ptr, seerep_hdf5_core::Hdf5CoreTf::PARENT_FRAME, tf.frame_id);
  writeAttributeToHdf5(*tf_group_ptr, seerep_hdf5_core::Hdf5CoreTf::CHILD_FRAME, tf.child_frame_id);

  // write time
  std::vector<int64_t> time;
  time.push_back(tf.seconds);
  time.push_back(tf.nanos);
  data_set_time_ptr->select({ size - 1, 0 }, { 1, 2 }).write(time);

  // write translation
  std::vector<double> trans;
  trans.push_back(tf.translation[0]);
  trans.push_back(tf.translation[1]);
  trans.push_back(tf.translation[2]);
  data_set_trans_ptr->select({ size - 1, 0 }, { 1, 3 }).write(trans);

  // write rotation
  std::vector<double> rot;
  rot.push_back(tf.rotation[0]);
  rot.push_back(tf.rotation[1]);
  rot.push_back(tf.rotation[2]);
  rot.push_back(tf.rotation[3]);
  data_set_rot_ptr->select({ size - 1, 0 }, { 1, 4 }).write(rot);

  m_file->flush();
}

std::vector<TfTransform> Hdf5PyTf::readTransformStamped(const std::string& frame_id)
{
  std::string hdf5_group_path = seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF + "/" + frame_id;
  std::string hdf5_dataset_time_path = hdf5_group_path + "/" + "time";
  std::string hdf5_dataset_trans_path = hdf5_group_path + "/" + "translation";
  std::string hdf5_dataset_rot_path = hdf5_group_path + "/" + "rotation";

  std::shared_ptr<HighFive::Group> tf_group_ptr = getHdf5Group(hdf5_group_path);
  std::shared_ptr<HighFive::DataSet> data_set_time_ptr = getHdf5DataSet(hdf5_dataset_time_path);
  std::shared_ptr<HighFive::DataSet> data_set_trans_ptr = getHdf5DataSet(hdf5_dataset_trans_path);
  std::shared_ptr<HighFive::DataSet> data_set_rot_ptr = getHdf5DataSet(hdf5_dataset_rot_path);

  if (tf_group_ptr == nullptr || data_set_time_ptr == nullptr || data_set_trans_ptr == nullptr ||
      data_set_rot_ptr == nullptr)
  {
    throw std::invalid_argument("unable to obtain the required tf hdf5 group or datasets for frame " + frame_id);
  }

  uint64_t size = 0;
  if (tf_group_ptr->hasAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE))
  {
    tf_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE).read(size);
  }

  if (size == 0)
  {
    throw std::invalid_argument("tf data has size 0");
  }

  // read frames
  if (!tf_group_ptr->hasAttribute(seerep_hdf5_core::Hdf5CoreTf::PARENT_FRAME) ||
      !tf_group_ptr->hasAttribute(seerep_hdf5_core::Hdf5CoreTf::CHILD_FRAME))
  {
    throw std::invalid_argument("parent frame or child frame not set for transform");
  }

  std::string frame_id_read;
  std::string child_frame_id;
  tf_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::PARENT_FRAME).read(frame_id_read);
  tf_group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::CHILD_FRAME).read(child_frame_id);

  if (frame_id_read.compare(frame_id) != 0)
  {
    throw std::invalid_argument("frame id of file (" + frame_id_read + ") does not match requested frame id (" +
                                frame_id + ")");
  }

  // read data
  std::vector<std::vector<int64_t>> time;
  std::vector<std::vector<double>> trans;
  std::vector<std::vector<double>> rot;
  data_set_time_ptr->read(time);
  data_set_trans_ptr->read(trans);
  data_set_rot_ptr->read(rot);

  // check if all have the right size
  if (time.size() != size || trans.size() != size || rot.size() != size)
  {
    throw std::invalid_argument("sizes of time (" + std::to_string(time.size()) + "), translation (" +
                                std::to_string(trans.size()) + ") and rotation (" + std::to_string(rot.size()) +
                                ") not matching. Size expected by value in metadata (" + std::to_string(size) + ")");
  }

  // assemble tf data
  std::vector<TfTransform> tfs;
  for (std::size_t i = 0; i < size; i++)
  {
    TfTransform tf;
    tf.frame_id = frame_id;
    tf.child_frame_id = child_frame_id;

    tf.seconds = time.at(i).at(0);
    tf.nanos = time.at(i).at(1);

    tf.translation[0] = trans.at(i).at(0);
    tf.translation[1] = trans.at(i).at(1);
    tf.translation[2] = trans.at(i).at(2);

    tf.rotation[0] = rot.at(i).at(0);
    tf.rotation[1] = rot.at(i).at(1);
    tf.rotation[2] = rot.at(i).at(2);
    tf.rotation[3] = rot.at(i).at(3);

    tfs.push_back(tf);
  }
  return tfs;
}

} /* namespace seerep_hdf5_py */
