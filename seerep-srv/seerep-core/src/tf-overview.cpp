#include "seerep-core/tf-overview.h"

namespace seerep_core
{
TFOverview::TFOverview()
{
}
TFOverview::TFOverview(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io) : m_hdf5_io(hdf5_io)
{
  recreateDatasets();
}
TFOverview::~TFOverview()
{
}

void TFOverview::recreateDatasets()
{
  std::vector<std::string> tfs = m_hdf5_io->getGroupDatasets("tf");
  for (auto name : tfs)
  {
    std::cout << "found " << name << " in HDF5 file." << std::endl;

    try
    {
      auto tf = std::make_shared<TF>(m_hdf5_io, name);

      addToIndices(tf);
    }
    catch (const std::runtime_error& e)
    {
      std::cout << e.what() << std::endl;
    }
  }
}

std::vector<std::optional<seerep::TransformStamped>>
TFOverview::getData(int64_t timesecs, int64_t timenanos, std::string parentFrame, std::string childFrame)
{
  std::vector<std::optional<seerep::TransformStamped>> result;

  return result;
}

void TFOverview::addDataset(const seerep::TransformStamped& transform)
{
  auto tf = std::make_shared<seerep_core::TF>(m_hdf5_io, transform);
  addToIndices(tf);
}

void TFOverview::addToIndices(std::shared_ptr<seerep_core::TF> tf)
{
  m_datasets.insert(std::make_pair(tf->getID(), tf));
}

} /* namespace seerep_core */
