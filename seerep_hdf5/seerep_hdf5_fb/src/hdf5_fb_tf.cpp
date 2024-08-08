#include "seerep_hdf5_fb/hdf5_fb_tf.h"

#include <highfive/H5DataSet.hpp>

#include "seerep_hdf5_core/hdf5_core_tf.h"

namespace seerep_hdf5_fb
{
Hdf5FbTf::Hdf5FbTf(std::shared_ptr<HighFive::File>& file,
                   std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx), Hdf5FbGeneral(file, write_mtx)
{
}

void Hdf5FbTf::writeTransformStamped(const seerep::fb::TransformStamped& tf)
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
  std::string parentFrame = tf.header()->frame_id()->str();
  std::string childFrame = tf.child_frame_id()->str();
  // replace the '/' of namespaces in the frameIds by '_' to avoid the creation
  // of subgroups in HDF5
  std::replace(parentFrame.begin(), parentFrame.end(), '/', '_');
  std::replace(childFrame.begin(), childFrame.end(), '/', '_');
  std::string hdf5DatasetPath =
      hdf5_group_tf + "/" + parentFrame + "_" + childFrame;
  std::string hdf5DatasetTimePath = hdf5DatasetPath + "/" + "time";
  std::string hdf5DatasetTransPath = hdf5DatasetPath + "/" + "translation";
  std::string hdf5DatasetRotPath = hdf5DatasetPath + "/" + "rotation";

  std::shared_ptr<HighFive::DataSet> data_set_time_ptr, data_set_trans_ptr,
      data_set_rot_ptr;
  uint64_t size = 0;

  if (!m_file->exist(hdf5DatasetPath))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "data id " << hdf5DatasetPath
        << " does not exist! Creat new dataset in hdf5";
    HighFive::Group group = m_file->createGroup(hdf5DatasetPath);
    group.createAttribute(seerep_hdf5_core::Hdf5CoreTf::CHILD_FRAME,
                          tf.child_frame_id()->str());
    group.createAttribute(seerep_hdf5_core::Hdf5CoreTf::PARENT_FRAME,
                          tf.header()->frame_id()->str());

    // TIME
    // Create a dataspace with initial shape and max shape
    HighFive::DataSpace data_space_time({ 1, 2 },
                                        { HighFive::DataSpace::UNLIMITED, 2 });
    // Use chunking
    HighFive::DataSetCreateProps props_time;
    props_time.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 2 }));
    data_set_time_ptr =
        std::make_shared<HighFive::DataSet>(m_file->createDataSet<int64_t>(
            hdf5DatasetTimePath, data_space_time, props_time));

    // TRANSLATION
    // Create a dataspace with initial shape and max shape
    HighFive::DataSpace data_space_trans({ 1, 3 },
                                         { HighFive::DataSpace::UNLIMITED, 3 });
    // Use chunking
    HighFive::DataSetCreateProps props_trans;
    props_trans.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 3 }));
    data_set_trans_ptr =
        std::make_shared<HighFive::DataSet>(m_file->createDataSet<double>(
            hdf5DatasetTransPath, data_space_trans, props_trans));

    // ROTATION
    // Create a dataspace with initial shape and max shape
    HighFive::DataSpace data_space_rot({ 1, 4 },
                                       { HighFive::DataSpace::UNLIMITED, 4 });
    // Use chunking
    HighFive::DataSetCreateProps props_rot;
    props_rot.add(HighFive::Chunking(std::vector<hsize_t>{ 1, 4 }));
    data_set_rot_ptr =
        std::make_shared<HighFive::DataSet>(m_file->createDataSet<double>(
            hdf5DatasetRotPath, data_space_rot, props_rot));
  }
  else
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
        << "data id " << hdf5DatasetPath << " already exists!";
    data_set_time_ptr = std::make_shared<HighFive::DataSet>(
        m_file->getDataSet(hdf5DatasetTimePath));
    data_set_trans_ptr = std::make_shared<HighFive::DataSet>(
        m_file->getDataSet(hdf5DatasetTransPath));
    data_set_rot_ptr = std::make_shared<HighFive::DataSet>(
        m_file->getDataSet(hdf5DatasetRotPath));

    HighFive::Group group = m_file->getGroup(hdf5DatasetPath);
    group.getAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE).read(size);

    // Resize the dataset to a larger size
    data_set_time_ptr->resize({ size + 1, 2 });
    data_set_trans_ptr->resize({ size + 1, 3 });
    data_set_rot_ptr->resize({ size + 1, 4 });
  }

  // write time
  std::vector<int64_t> time;
  time.push_back(tf.header()->stamp()->seconds());
  time.push_back(tf.header()->stamp()->nanos());
  data_set_time_ptr->select({ size, 0 }, { 1, 2 }).write(time);

  // write translation
  std::vector<double> trans;
  trans.push_back(tf.transform()->translation()->x());
  trans.push_back(tf.transform()->translation()->y());
  trans.push_back(tf.transform()->translation()->z());
  data_set_trans_ptr->select({ size, 0 }, { 1, 3 }).write(trans);

  // write rotation
  std::vector<double> rot;
  rot.push_back(tf.transform()->rotation()->x());
  rot.push_back(tf.transform()->rotation()->y());
  rot.push_back(tf.transform()->rotation()->z());
  rot.push_back(tf.transform()->rotation()->w());
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

bool Hdf5FbTf::deleteTransformStamped(std::string parentFrameId,
                                      std::string childFrameId,
                                      const bool isStatic,
                                      const seerep::fb::Timestamp& timeMin,
                                      const seerep::fb::Timestamp& timeMax) const
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

  std::replace(parentFrameId.begin(), parentFrameId.end(), '/', '_');
  std::replace(childFrameId.begin(), childFrameId.end(), '/', '_');

  std::string hdf5GroupPath =
      hdf5_group_tf + "/" + parentFrameId + "_" + childFrameId;
  std::string hdf5DatasetTimePath = hdf5GroupPath + "/" + "time";
  std::string hdf5DatasetTransPath = hdf5GroupPath + "/" + "translation";
  std::string hdf5DatasetRotPath = hdf5GroupPath + "/" + "rotation";

  if (!m_file->exist(hdf5GroupPath) || !m_file->exist(hdf5DatasetTimePath) ||
      !m_file->exist(hdf5DatasetTransPath) ||
      !m_file->exist(hdf5DatasetRotPath))
  {
    auto str_static = isStatic ? "static" : "non-static";

    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "No " << str_static << " tfs matching the frames (" << parentFrameId
        << " -> " << childFrameId << ") were found!";
    return false;
  }

  std::shared_ptr<HighFive::Group> group_ptr =
      std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));

  // read size
  if (!group_ptr->hasAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE))
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
        << "Could not access size attribute of the tf group " << parentFrameId
        << "_" << childFrameId << "!";
    return false;
  }

  long unsigned int size;
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE).read(size);

  if (size == 0)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "tf data has size 0.";
    return false;
  }

  auto data_set_time_ptr = std::make_shared<HighFive::DataSet>(
      m_file->getDataSet(hdf5DatasetTimePath));
  auto data_set_trans_ptr = std::make_shared<HighFive::DataSet>(
      m_file->getDataSet(hdf5DatasetTransPath));
  auto data_set_rot_ptr = std::make_shared<HighFive::DataSet>(
      m_file->getDataSet(hdf5DatasetRotPath));

  std::vector<std::vector<int64_t>> time;
  data_set_time_ptr->read(time);

  std::vector<std::vector<double>> trans;
  data_set_trans_ptr->read(trans);

  std::vector<std::vector<double>> rot;
  data_set_rot_ptr->read(rot);

  // check if all have the right size
  if (time.size() != size || trans.size() != size || rot.size() != size)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "sizes of time (" << time.size() << "), translation ("
        << trans.size() << ") and rotation (" << rot.size()
        << ") not matching. Size expected by value in metadata (" << size
        << ")";
    return false;
  }

  // sort indices by time
  // initialize original index locations
  std::vector<size_t> idx(time.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in window
  // using std::stable_sort instead of std::sort
  // to avoid unnecessary index re-orderings
  // when window contains elements of equal values
  stable_sort(idx.begin(), idx.end(), [&time](size_t i1, size_t i2) {
    return time[i1][0] < time[i2][0] ||
           (time[i1][0] == time[i2][0] && time[i1][1] < time[i2][1]);
  });

  // inclusive first index search
  auto timeMinIt =
      std::find_if(idx.begin(), idx.end(), [&time, &timeMin](size_t i) {
        return time[i][0] > timeMin.seconds() ||
               (time[i][0] == timeMin.seconds() &&
                time[i][1] >= timeMin.nanos());
      });

  // exclusive last index search
  auto timeMaxIt =
      std::find_if(idx.begin(), idx.end(), [&time, &timeMax](size_t i) {
        return time[i][0] > timeMax.seconds() ||
               (time[i][0] == timeMax.seconds() &&
                time[i][1] >= timeMax.nanos());
      });

  // first inclusive index
  auto firstDelIdx = std::distance(idx.begin(), timeMinIt);
  // last exclusive index
  auto endDelIdx = std::distance(idx.begin(), timeMaxIt);

  // early abort if no transform is between the provided time interval
  if (timeMinIt == idx.end() || firstDelIdx >= endDelIdx)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "No tf was found in the timeinterval between " << timeMin.seconds()
        << "s / " << timeMin.nanos() << "ns (inclusive) and "
        << timeMax.seconds() << "s / " << timeMax.nanos() << "ns (exclusive)!";
    return false;
  }
  std::vector<std::vector<int64_t>> reduced_time;
  std::vector<std::vector<double>> reduced_trans;
  std::vector<std::vector<double>> reduced_rot;

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
      << "Deleting tfs from " << timeMin.seconds() << "s / " << timeMin.nanos()
      << "ns (inclusive) up to " << timeMax.seconds() << "s / "
      << timeMax.nanos() << "ns (exclusive)...";

  for (size_t i = 0; i < idx.size(); ++i)
  {
    // ignore all elements in the timeinterval
    if (i == (size_t)firstDelIdx)
    {
      if ((size_t)endDelIdx == idx.size())
      {
        break;
      }
      i = endDelIdx;
    }
    reduced_time.push_back(time[idx[i]]);
    reduced_trans.push_back(trans[idx[i]]);
    reduced_rot.push_back(rot[idx[i]]);
  }

  auto reduced_size = reduced_time.size();

  data_set_time_ptr->resize({ reduced_size, 2 });
  data_set_time_ptr->select({ 0, 0 }, { reduced_size, 2 }).write(reduced_time);

  data_set_trans_ptr->resize({ reduced_size, 3 });
  data_set_trans_ptr->select({ 0, 0 }, { reduced_size, 3 }).write(reduced_trans);

  data_set_rot_ptr->resize({ reduced_size, 4 });
  data_set_rot_ptr->select({ 0, 0 }, { reduced_size, 4 }).write(reduced_rot);

  // write the size as group attribute
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE).write(reduced_size);

  m_file->flush();
  return true;
}

std::optional<std::vector<flatbuffers::Offset<seerep::fb::TransformStamped>>>
Hdf5FbTf::readTransformStamped(const std::string& id, const bool isStatic)
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

  if (!m_file->exist(hdf5GroupPath) || !m_file->exist(hdf5DatasetTimePath) ||
      !m_file->exist(hdf5DatasetTransPath) ||
      !m_file->exist(hdf5DatasetRotPath))
  {
    return std::nullopt;
  }

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
      << "loading " << hdf5GroupPath;

  // read size
  std::shared_ptr<HighFive::Group> group_ptr =
      std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));
  long unsigned int size;
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::SIZE).read(size);
  if (size == 0)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "tf data has size 0.";
    return std::nullopt;
  }

  // read frames
  std::string parentframe;
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::PARENT_FRAME)
      .read(parentframe);
  std::string childframe;
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::CHILD_FRAME)
      .read(childframe);

  // read time
  std::shared_ptr<HighFive::DataSet> data_set_time_ptr =
      std::make_shared<HighFive::DataSet>(
          m_file->getDataSet(hdf5DatasetTimePath));
  std::vector<std::vector<int64_t>> time;
  data_set_time_ptr->read(time);

  // read translation
  std::shared_ptr<HighFive::DataSet> data_set_trans_ptr =
      std::make_shared<HighFive::DataSet>(
          m_file->getDataSet(hdf5DatasetTransPath));
  std::vector<std::vector<double>> trans;
  data_set_trans_ptr->read(trans);

  // read rotation
  std::shared_ptr<HighFive::DataSet> data_set_rot_ptr =
      std::make_shared<HighFive::DataSet>(
          m_file->getDataSet(hdf5DatasetRotPath));
  std::vector<std::vector<double>> rot;
  data_set_rot_ptr->read(rot);

  // check if all have the right size
  if (time.size() != size || trans.size() != size || rot.size() != size)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::warning)
        << "sizes of time (" << time.size() << "), translation ("
        << trans.size() << ") and rotation (" << rot.size()
        << ") not matching. Size expected by value in metadata (" << size
        << ")";
    return std::nullopt;
  }

  std::vector<flatbuffers::Offset<seerep::fb::TransformStamped>> tfs;
  flatbuffers::FlatBufferBuilder builder;
  seerep::fb::TransformStampedBuilder tfBuilder(builder);
  for (long unsigned int i = 0; i < size; i++)
  {
    tfBuilder.add_child_frame_id(builder.CreateString(childframe));
    tfBuilder.add_header(seerep::fb::CreateHeader(
        builder, 0,
        seerep::fb::CreateTimestamp(builder, time.at(i).at(0), time.at(i).at(1)),
        builder.CreateString(parentframe), builder.CreateString(""),
        builder.CreateString("")));

    tfBuilder.add_transform(seerep::fb::CreateTransform(
        builder,
        seerep::fb::CreateVector3(builder, trans.at(i).at(0), trans.at(i).at(1),
                                  trans.at(i).at(2)),
        seerep::fb::CreateQuaternion(builder, rot.at(i).at(0), rot.at(i).at(1),
                                     rot.at(i).at(2))));

    tfs.push_back(tfBuilder.Finish());
  }
  return tfs;
}

std::optional<std::vector<std::string>>
Hdf5FbTf::readTransformStampedFrames(const std::string& id, const bool isStatic)
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

  std::shared_ptr<HighFive::Group> group_ptr =
      std::make_shared<HighFive::Group>(m_file->getGroup(hdf5GroupPath));

  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info)
      << "loading parent frame of " << hdf5GroupPath;

  // read frames
  std::string parentframe;
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::PARENT_FRAME)
      .read(parentframe);
  std::string childframe;
  group_ptr->getAttribute(seerep_hdf5_core::Hdf5CoreTf::CHILD_FRAME)
      .read(childframe);

  return std::vector<std::string>{ parentframe, childframe };
}

}  // namespace seerep_hdf5_fb
