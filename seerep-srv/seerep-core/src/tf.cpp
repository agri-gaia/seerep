#include "seerep-core/tf.h"

namespace seerep_core
{
// constructor when data received and stored to hdf5
TF::TF(std::shared_ptr<seerep_pb_io::TfIO> hdf5_io, const seerep::TransformStamped& tf)
  : m_hdf5_io(hdf5_io)
  , m_id(TF::idFromFrameNames(tf.header().frame_id(), tf.child_frame_id()))
  , m_parentframe(tf.header().frame_id())
  , m_childframe(tf.child_frame_id())
{
  m_hdf5_io->writeTransformStamped(tf);
}

// constructor if recreating the server from hdf5
TF::TF(std::shared_ptr<seerep_pb_io::TfIO> hdf5_io, const std::string& id) : m_hdf5_io(hdf5_io), m_id(id)
{
  std::optional<std::vector<std::string>> frames = m_hdf5_io->readTransformStampedFrames(id);

  if (frames)
  {
    m_parentframe = frames.value().at(0);
    m_childframe = frames.value().at(1);
  }
}

TF::~TF()
{
}

void TF::addData(const seerep::TransformStamped& tf)
{
  m_hdf5_io->writeTransformStamped(tf);
}

std::optional<std::vector<seerep::TransformStamped>> TF::getData()
{
  std::cout << "loading tf from tfs/" << m_id << std::endl;

  return m_hdf5_io->readTransformStamped(m_id);
}

std::string TF::getParentFrame()
{
  return m_parentframe;
}

std::string TF::getChildFrame()
{
  return m_childframe;
}

std::string TF::getID()
{
  return m_id;
}

std::string TF::idFromFrameNames(const std::string& parentframe, const std::string& childframe)
{
  return parentframe + "_" + childframe;
}

} /* namespace seerep_core */
