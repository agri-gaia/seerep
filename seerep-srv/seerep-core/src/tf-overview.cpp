#include "seerep-core/tf-overview.h"

namespace seerep_core
{
// construct tfbuffer with INT_MAX so that it holds ALL tfs added
TFOverview::TFOverview(std::shared_ptr<seerep_hdf5::SeerepHDF5IO> hdf5_io)
  : m_hdf5_io(hdf5_io), tfbuffer(ros::DURATION_MAX)
{
  recreateDatasets();
}
TFOverview::~TFOverview()
{
}

void TFOverview::recreateDatasets()
{
  std::vector<std::string> tfs = m_hdf5_io->getGroupDatasets("tf");
  for (auto const& name : tfs)
  {
    std::cout << "found " << name << " in HDF5 file." << std::endl;

    try
    {
      auto tf = std::make_shared<TF>(m_hdf5_io, name);

      addToIndices(tf);

      std::optional<std::vector<seerep::TransformStamped>> transforms = tf->getData();
      if (transforms)
      {
        for (auto& transform : transforms.value())

          addToTfBuffer(transform);
      }
    }
    catch (const std::runtime_error& e)
    {
      std::cout << e.what() << std::endl;
    }
  }
}

std::optional<seerep::TransformStamped> TFOverview::getData(int64_t timesecs, int64_t timenanos,
                                                            std::string targetFrame, std::string sourceFrame)
{
  // if (!ros::Time::isValid())
  // {
  //   ros::Time::init();
  // }

  try
  {
    return seerep_ros_conversions::toProto(
        tfbuffer.lookupTransform(targetFrame, sourceFrame, ros::Time(timesecs, timenanos)));
  }
  catch (const std::exception& e)
  {
    std::cout << e.what() << std::endl;
    return std::nullopt;
  }
}

void TFOverview::addDataset(const seerep::TransformStamped& transform)
{
  addToTfBuffer(transform);

  auto tfInMap = m_datasets.find(TF::idFromFrameNames(transform.header().frame_id(), transform.child_frame_id()));
  if (tfInMap == m_datasets.end())
  {
    // create new tf object and add to map if not existing yet
    auto tf = std::make_shared<seerep_core::TF>(m_hdf5_io, transform);
    addToIndices(tf);
  }
  else
  {
    // add transform to tf object
    tfInMap->second->addData(transform);
  }
}

AabbHierarchy::AABB TFOverview::transformAABB(AabbHierarchy::AABB aabb, std::string sourceFrame, std::string targetFrame)
{
}

void TFOverview::addToIndices(std::shared_ptr<seerep_core::TF> tf)
{
  m_datasets.insert(std::make_pair(tf->getID(), tf));
}

void TFOverview::addToTfBuffer(seerep::TransformStamped transform)
{
  tfbuffer.setTransform(seerep_ros_conversions::toROS(transform), "fromHDF5");
}

// for (auto tf : m_datasets)
// {
//   std::optional<std::vector<seerep::TransformStamped>> transforms = tf.second->getData();

//   for (auto transform : transforms.value())
//   {
//     tfbuffer.setTransform(seerep_ros_conversions::toROS(transform), "fromHDF5");
//   }

//   std::cout << "loaded from hdf5: " << std::endl tfbuffer.allFramesAsString() << std::endl;
// }

} /* namespace seerep_core */
