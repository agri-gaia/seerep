#include "seerep-hdf5-py/hdf5-py-tf.h"

#include <highfive/H5DataSet.hpp>

#include "seerep-hdf5-py/hdf5-py.h"

namespace seerep_hdf5_py
{

Hdf5PyTf::Hdf5PyTf(Hdf5FileWrapper& hdf5_file)
  : Hdf5CoreGeneral(hdf5_file.getFile(), hdf5_file.getMutex())
  // , Hdf5CoreTf(hdf5_file.getFile(), hdf5_file.getMutex())
  , Hdf5PyGeneral(hdf5_file)
{
}

} /* namespace seerep_hdf5_py */
