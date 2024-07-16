#include "seerep_hdf5_py/hdf5_py_tf.h"

#include <highfive/H5DataSet.hpp>

#include "seerep_hdf5_py/hdf5_py.h"

namespace seerep_hdf5_py
{

Hdf5PyTf::Hdf5PyTf(Hdf5FileWrapper& hdf5File)
  : Hdf5CoreGeneral(hdf5File.getFile(), hdf5File.getMutex())
  , Hdf5PyGeneral(hdf5File)
{
}

void Hdf5PyTf::writeTransformStamped(const TfTransform& tf)
{
  std::string hdf5GroupPath = seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF +
                              "/" + tf.frameId_ + "_" + tf.childFrameId_;
  std::string hdf5DatasetTimePath = hdf5GroupPath + "/" + "time";
  std::string hdf5DatasetTransPath = hdf5GroupPath + "/" + "translation";
  std::string hdf5DatasetRotPath = hdf5GroupPath + "/" + "rotation";

  std::shared_ptr<HighFive::Group> tfGroupPtr = getHdf5Group(hdf5GroupPath);
  if (tfGroupPtr == nullptr)
  {
    throw std::invalid_argument("unable to obtain tf hdf5 group for frame " +
                                tf.frameId_);
  }

  HighFive::DataSpace dataSpaceTime({ 1, 2 },
                                    { HighFive::DataSpace::UNLIMITED, 2 });
  HighFive::DataSetCreateProps propsTime;
  propsTime.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 2 }));
  std::shared_ptr<HighFive::DataSet> datasetTimePtr =
      getHdf5DataSet<int64_t>(hdf5DatasetTimePath, dataSpaceTime, propsTime);

  HighFive::DataSpace dataSpaceTrans({ 1, 3 },
                                     { HighFive::DataSpace::UNLIMITED, 3 });
  HighFive::DataSetCreateProps propsTrans;
  propsTrans.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 3 }));
  std::shared_ptr<HighFive::DataSet> datasetTransPtr =
      getHdf5DataSet<double>(hdf5DatasetTransPath, dataSpaceTrans, propsTrans);

  HighFive::DataSpace dataSpaceRot({ 1, 4 },
                                   { HighFive::DataSpace::UNLIMITED, 4 });
  HighFive::DataSetCreateProps propsRot;
  propsRot.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 4 }));
  std::shared_ptr<HighFive::DataSet> datasetRotPtr =
      getHdf5DataSet<double>(hdf5DatasetRotPath, dataSpaceRot, propsRot);

  if (datasetTimePtr == nullptr || datasetTransPtr == nullptr ||
      datasetRotPtr == nullptr)
  {
    throw std::invalid_argument(
        "unable to obtain the required tf hdf5 datasets for frame " +
        tf.frameId_);
  }

  uint64_t size = 1;
  if (tfGroupPtr->hasAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE))
  {
    tfGroupPtr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE).read(size);

    size++;

    datasetTimePtr->resize({ size, 2 });
    datasetTransPtr->resize({ size, 3 });
    datasetRotPtr->resize({ size, 4 });

    tfGroupPtr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE).write(size);
  }
  else
  {
    tfGroupPtr->createAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE, size);
  }

  // write frames
  writeAttributeToHdf5(*tfGroupPtr, seerep_hdf5_core::Hdf5CoreTf::PARENT_FRAME,
                       tf.frameId_);
  writeAttributeToHdf5(*tfGroupPtr, seerep_hdf5_core::Hdf5CoreTf::CHILD_FRAME,
                       tf.childFrameId_);

  // write time
  std::vector<int64_t> time;
  time.push_back(tf.seconds_);
  time.push_back(tf.nanos_);
  datasetTimePtr->select({ size - 1, 0 }, { 1, 2 }).write(time);

  // write translation
  std::vector<double> trans;
  trans.push_back(tf.translation_[0]);
  trans.push_back(tf.translation_[1]);
  trans.push_back(tf.translation_[2]);
  datasetTransPtr->select({ size - 1, 0 }, { 1, 3 }).write(trans);

  // write rotation
  std::vector<double> rot;
  rot.push_back(tf.rotation_[0]);
  rot.push_back(tf.rotation_[1]);
  rot.push_back(tf.rotation_[2]);
  rot.push_back(tf.rotation_[3]);
  datasetRotPtr->select({ size - 1, 0 }, { 1, 4 }).write(rot);

  m_file->flush();
}

std::vector<TfTransform>
Hdf5PyTf::readTransformStamped(const std::string& frameId)
{
  const std::scoped_lock lock(*m_write_mtx);

  if (!m_file->exist(seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF))
  {
    throw std::invalid_argument("no tf data found");
  }

  std::vector<TfTransform> tfs;

  const HighFive::Group& tfRootGroup =
      m_file->getGroup(seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF);
  for (const std::string& tfEntry : tfRootGroup.listObjectNames())
  {
    if (tfRootGroup.getObjectType(tfEntry) == HighFive::ObjectType::Group)
    {
      const HighFive::Group& tfGroup = tfRootGroup.getGroup(tfEntry);

      if (!tfGroup.hasAttribute(seerep_hdf5_core::Hdf5CoreTf::CHILD_FRAME))
      {
        continue;
      }

      std::string groupChildFrameId;
      tfGroup.getAttribute(seerep_hdf5_core::Hdf5CoreTf::CHILD_FRAME)
          .read(groupChildFrameId);

      if (groupChildFrameId.compare(frameId) == 0)
      {
        std::vector<TfTransform> tfs_part = readGroupTransformStamped(tfEntry);
        tfs.insert(tfs.end(), tfs_part.begin(), tfs_part.end());
      }
    }
  }

  return tfs;
}

std::vector<TfTransform>
Hdf5PyTf::readGroupTransformStamped(const std::string& tfGroupId)
{
  std::string hdf5GroupPath =
      seerep_hdf5_core::Hdf5CoreTf::HDF5_GROUP_TF + "/" + tfGroupId;
  std::string hdf5DatasetTimePath = hdf5GroupPath + "/" + "time";
  std::string hdf5DatasetTransPath = hdf5GroupPath + "/" + "translation";
  std::string hdf5DatasetRotPath = hdf5GroupPath + "/" + "rotation";

  std::shared_ptr<HighFive::Group> tfGroupPtr = getHdf5Group(hdf5GroupPath);
  std::shared_ptr<HighFive::DataSet> datasetTimePtr =
      getHdf5DataSet(hdf5DatasetTimePath);
  std::shared_ptr<HighFive::DataSet> datasetTransPtr =
      getHdf5DataSet(hdf5DatasetTransPath);
  std::shared_ptr<HighFive::DataSet> datasetRotPtr =
      getHdf5DataSet(hdf5DatasetRotPath);

  if (tfGroupPtr == nullptr || datasetTimePtr == nullptr ||
      datasetTransPtr == nullptr || datasetRotPtr == nullptr)
  {
    throw std::invalid_argument(
        "unable to obtain the required tf hdf5 group or datasets " + tfGroupId);
  }

  uint64_t size = 0;
  if (tfGroupPtr->hasAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE))
  {
    tfGroupPtr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE).read(size);
  }

  if (size == 0)
  {
    throw std::invalid_argument("tf data has size 0");
  }

  // read frames
  if (!tfGroupPtr->hasAttribute(seerep_hdf5_core::Hdf5CoreTf::PARENT_FRAME) ||
      !tfGroupPtr->hasAttribute(seerep_hdf5_core::Hdf5CoreTf::CHILD_FRAME))
  {
    throw std::invalid_argument(
        "parent frame or child frame not set for transform");
  }

  std::string frameId;
  std::string childFrameId;
  tfGroupPtr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::PARENT_FRAME)
      .read(frameId);
  tfGroupPtr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::CHILD_FRAME)
      .read(childFrameId);

  // read data
  std::vector<std::vector<int64_t>> time;
  std::vector<std::vector<double>> trans;
  std::vector<std::vector<double>> rot;
  datasetTimePtr->read(time);
  datasetTransPtr->read(trans);
  datasetRotPtr->read(rot);

  // check if all have the right size
  if (time.size() != size || trans.size() != size || rot.size() != size)
  {
    throw std::invalid_argument(
        "sizes of time (" + std::to_string(time.size()) + "), translation (" +
        std::to_string(trans.size()) + ") and rotation (" +
        std::to_string(rot.size()) +
        ") not matching. Size expected by value in metadata (" +
        std::to_string(size) + ")");
  }

  // assemble tf data
  std::vector<TfTransform> tfs;
  for (std::size_t i = 0; i < size; i++)
  {
    TfTransform tf;
    tf.frameId_ = frameId;
    tf.childFrameId_ = childFrameId;

    tf.seconds_ = time.at(i).at(0);
    tf.nanos_ = time.at(i).at(1);

    tf.translation_[0] = trans.at(i).at(0);
    tf.translation_[1] = trans.at(i).at(1);
    tf.translation_[2] = trans.at(i).at(2);

    tf.rotation_[0] = rot.at(i).at(0);
    tf.rotation_[1] = rot.at(i).at(1);
    tf.rotation_[2] = rot.at(i).at(2);
    tf.rotation_[3] = rot.at(i).at(3);

    tfs.push_back(tf);
  }
  return tfs;
}

} /* namespace seerep_hdf5_py */
