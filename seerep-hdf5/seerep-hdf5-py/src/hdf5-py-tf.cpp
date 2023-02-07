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
  std::string hdf5DatasetPath = HDF5_GROUP_TF + "/" + tf.frame_id + "_" + tf.child_frame_id;
  std::string hdf5DatasetTimePath = hdf5DatasetPath + "/" + "time";
  std::string hdf5DatasetTransPath = hdf5DatasetPath + "/" + "translation";
  std::string hdf5DatasetRotPath = hdf5DatasetPath + "/" + "rotation";

  std::shared_ptr<HighFive::Group> tf_group_ptr = getHdf5Group(hdf5DatasetPath);
  if (tf_group_ptr == nullptr)
  {
    throw std::invalid_argument("unable to obtain tf hdf5 group for frame " + tf.frame_id + " and child frame " +
                                tf.child_frame_id);
  }

  uint64_t size = 0;
  if (tf_group_ptr->hasAttribute(SIZE))
  {
    tf_group_ptr->getAttribute(SIZE).read(size);
  }

  HighFive::DataSpace data_space_time({ 1, 2 }, { HighFive::DataSpace::UNLIMITED, 2 });
  std::shared_ptr<HighFive::DataSet> data_set_time_ptr = getHdf5DataSet<int64_t>(hdf5DatasetTimePath, data_space_time);

  HighFive::DataSpace data_space_trans({ 1, 3 }, { HighFive::DataSpace::UNLIMITED, 3 });
  std::shared_ptr<HighFive::DataSet> data_set_trans_ptr =
      getHdf5DataSet<double>(hdf5DatasetTransPath, data_space_trans);

  HighFive::DataSpace data_space_rot({ 1, 4 }, { HighFive::DataSpace::UNLIMITED, 4 });
  std::shared_ptr<HighFive::DataSet> data_set_rot_ptr = getHdf5DataSet<double>(hdf5DatasetRotPath, data_space_rot);

  if (data_set_time_ptr == nullptr || data_set_trans_ptr == nullptr || data_set_rot_ptr == nullptr)
  {
    throw std::invalid_argument("unable to obtain the required tf hdf5 datasets for frame " + tf.frame_id +
                                " and child frame " + tf.child_frame_id);
  }

  data_set_time_ptr->resize({ size + 1, 2 });
  data_set_trans_ptr->resize({ size + 1, 3 });
  data_set_rot_ptr->resize({ size + 1, 4 });

  // write the size as group attribute
  if (!tf_group_ptr->hasAttribute(SIZE))
  {
    tf_group_ptr->createAttribute(SIZE, ++size);
  }
  else
  {
    tf_group_ptr->getAttribute(SIZE).write(++size);
  }

  // write frames
  if (!tf_group_ptr->hasAttribute(PARENT_FRAME))
  {
    tf_group_ptr->createAttribute(PARENT_FRAME, tf.frame_id);
  }
  else
  {
    tf_group_ptr->getAttribute(PARENT_FRAME).write(tf.frame_id);
  }

  if (!tf_group_ptr->hasAttribute(CHILD_FRAME))
  {
    tf_group_ptr->createAttribute(CHILD_FRAME, tf.child_frame_id);
  }
  else
  {
    tf_group_ptr->getAttribute(CHILD_FRAME).write(tf.child_frame_id);
  }

  // write time
  std::vector<int64_t> time;
  time.push_back(tf.seconds);
  time.push_back(tf.nanos);
  data_set_time_ptr->select({ size, 0 }, { 1, 2 }).write(time);

  // write translation
  std::vector<double> trans;
  trans.push_back(tf.translation[0]);
  trans.push_back(tf.translation[1]);
  trans.push_back(tf.translation[2]);
  data_set_trans_ptr->select({ size, 0 }, { 1, 3 }).write(trans);

  // write rotation
  std::vector<double> rot;
  rot.push_back(tf.rotation[0]);
  rot.push_back(tf.rotation[1]);
  rot.push_back(tf.rotation[2]);
  rot.push_back(tf.rotation[3]);
  data_set_rot_ptr->select({ size, 0 }, { 1, 4 }).write(rot);

  m_file->flush();
}

} /* namespace seerep_hdf5_py */
