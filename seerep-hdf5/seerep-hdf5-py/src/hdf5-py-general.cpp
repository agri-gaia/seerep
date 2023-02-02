#include "seerep-hdf5-py/hdf5-py-general.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_py
{

Hdf5PyGeneral::Hdf5PyGeneral(Hdf5FileWrapper& hdf5_file) : Hdf5CoreGeneral(hdf5_file.getFile(), hdf5_file.getMutex())
{
}

void Hdf5PyGeneral::writeLabelsGeneral(const std::string& data_group_id, const std::string& uuid,
                                       const std::vector<seerep_hdf5_py::GeneralLabel>& general_labels)
{
}

} /* namespace seerep_hdf5_py */
