#include "seerep-core/tf.h"

namespace seerep_core
{
// constructor when data received and stored to hdf5
TF::TF(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io, const seerep::TransformStamped& tf)
  : m_hdf5_io(hdf5_io)
  , m_id(tf.header().frame_id() + "_" + tf.child_frame_id())
  , m_parentframe(tf.header().frame_id())
  , m_childframe(tf.child_frame_id())
{
  m_hdf5_io->writeTransformStamped(tf);
}

// constructor if recreating the server from hdf5
TF::TF(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io, const std::string& id) : m_hdf5_io(hdf5_io), m_id(id)
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

} /* namespace seerep_core */