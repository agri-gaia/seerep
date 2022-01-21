#include "seerep-core/tf.h"

namespace seerep_core
{
// constructor when data received and stored to hdf5
TF::TF(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io, const seerep::TransformStamped& tf)
  : m_hdf5_io(hdf5_io), m_id(tf.header().frame_id() + "_" + tf.child_frame_id())
{
  m_hdf5_io->writeTransformStamped(tf);
}

// constructor if recreating the server from hdf5
TF::TF(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io, const std::string& id) : m_hdf5_io(hdf5_io), m_id(id)
{
}

TF::~TF()
{
}

std::optional<std::vector<seerep::TransformStamped>> TF::getData()
{
  std::cout << "loading image from images/" << m_id << std::endl;

  std::optional<std::vector<seerep::TransformStamped>>
      tf;  //= m_hdf5_io->readImage(boost::lexical_cast<std::string>(m_uuid));

  return tf;
}

std::string TF::getID()
{
  return m_id;
}

} /* namespace seerep_core */
