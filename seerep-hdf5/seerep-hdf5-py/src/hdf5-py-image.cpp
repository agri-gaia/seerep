#include "seerep-hdf5-py/hdf5-py-image.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_py
{

Hdf5PyImage::Hdf5PyImage(Hdf5FileWrapper& hdf5_file)
  : Hdf5CoreGeneral(hdf5_file.getFile(), hdf5_file.getMutex())
  , Hdf5CoreImage(hdf5_file.getFile(), hdf5_file.getMutex())
  , Hdf5PyGeneral(hdf5_file)
{
}

void Hdf5PyImage::writeImage(const std::string& uuid, const std::string& frame_id, int64_t seconds, int32_t nanos,
                             uint32_t sequence, const py::array& image)
{
  // TODO
}

py::array Hdf5PyImage::readImage(const std::string& uuid)
{
  // TODO
}

} /* namespace seerep_hdf5_py */
