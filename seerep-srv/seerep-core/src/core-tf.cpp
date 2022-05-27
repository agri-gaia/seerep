#include "seerep-core/core-tf.h"

namespace seerep_core
{
// construct tfbuffer with INT_MAX so that it holds ALL tfs added
CoreTf::CoreTf(std::shared_ptr<seerep_hdf5_core::Hdf5CoreTf> hdf5_io)
  : m_hdf5_io(hdf5_io), m_tfbuffer(ros::DURATION_MAX)
{
  recreateDatasets();
}
CoreTf::~CoreTf()
{
}

void CoreTf::recreateDatasets()
{
  std::vector<std::string> tfs = m_hdf5_io->getGroupDatasets("tf");
  for (auto const& name : tfs)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << "found " << name << " in HDF5 file.";

    try
    {
      std::optional<std::vector<geometry_msgs::TransformStamped>> transforms = m_hdf5_io->readTransformStamped(name);
      if (transforms)
      {
        for (auto& transform : transforms.value())

          addToTfBuffer(transform);
      }
    }
    catch (const std::runtime_error& e)
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << e.what();
    }
  }
}

std::optional<geometry_msgs::TransformStamped> CoreTf::getData(const int64_t& timesecs, const int64_t& timenanos,
                                                               const std::string& targetFrame,
                                                               const std::string& sourceFrame)
{
  try
  {
    return m_tfbuffer.lookupTransform(targetFrame, sourceFrame, ros::Time(timesecs, timenanos));
  }
  catch (const std::exception& e)
  {
    BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::info) << e.what();
    return std::nullopt;
  }
}

void CoreTf::addDataset(const geometry_msgs::TransformStamped& transform)
{
  addToTfBuffer(transform);
}

// TODO optimise!
// TODO same as transformQuery -> merge!
seerep_core_msgs::AABB CoreTf::transformAABB(seerep_core_msgs::AABB aabb, const std::string& sourceFrame,
                                             const std::string& targetFrame, const int64_t& timeSecs,
                                             const int64_t& timeNanos)
{
  if (targetFrame != sourceFrame)
  {
    auto tf = m_tfbuffer.lookupTransform(targetFrame, sourceFrame, ros::Time(timeSecs, timeNanos));
    tf2::Transform transform;
    transform.setOrigin(
        tf2::Vector3(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z));
    transform.setRotation(tf2::Quaternion(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
                                          tf.transform.rotation.w));

    tf2::Vector3 vmin(bg::get<bg::min_corner, 0>(aabb), bg::get<bg::min_corner, 1>(aabb),
                      bg::get<bg::min_corner, 2>(aabb));
    tf2::Vector3 vmintransformed = transform * vmin;
    bg::set<bg::min_corner, 0>(aabb, vmintransformed.getX());
    bg::set<bg::min_corner, 1>(aabb, vmintransformed.getY());
    bg::set<bg::min_corner, 2>(aabb, vmintransformed.getZ());

    tf2::Vector3 vmax(bg::get<bg::max_corner, 0>(aabb), bg::get<bg::max_corner, 1>(aabb),
                      bg::get<bg::max_corner, 2>(aabb));
    tf2::Vector3 vmaxtransformed = transform * vmax;
    bg::set<bg::max_corner, 0>(aabb, vmaxtransformed.getX());
    bg::set<bg::max_corner, 1>(aabb, vmaxtransformed.getY());
    bg::set<bg::max_corner, 2>(aabb, vmaxtransformed.getZ());
  }
  return aabb;
}

bool CoreTf::canTransform(const std::string& sourceFrame, const std::string& targetFrame, const int64_t& timeSecs,
                          const int64_t& timeNanos)
{
  return m_tfbuffer.canTransform(targetFrame, sourceFrame, ros::Time(timeSecs, timeNanos));
}

// TODO same as transformAABB -> merge!
seerep_core_msgs::Query CoreTf::transformQuery(const seerep_core_msgs::Query& query, std::string targetFrame)
{
  if (targetFrame != query.header.frameId)
  {
    seerep_core_msgs::Query queryTransformed(query);

    auto tf = m_tfbuffer.lookupTransform(targetFrame, query.header.frameId,
                                         ros::Time(query.header.timestamp.seconds, query.header.timestamp.nanos));
    tf2::Transform transform;
    transform.setOrigin(
        tf2::Vector3(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z));
    transform.setRotation(tf2::Quaternion(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
                                          tf.transform.rotation.w));

    tf2::Vector3 vmin(bg::get<bg::min_corner, 0>(query.boundingbox), bg::get<bg::min_corner, 1>(query.boundingbox),
                      bg::get<bg::min_corner, 2>(query.boundingbox));
    tf2::Vector3 vmintransformed = transform * vmin;
    bg::set<bg::min_corner, 0>(queryTransformed.boundingbox, vmintransformed.getX());
    bg::set<bg::min_corner, 1>(queryTransformed.boundingbox, vmintransformed.getY());
    bg::set<bg::min_corner, 2>(queryTransformed.boundingbox, vmintransformed.getZ());

    tf2::Vector3 vmax(bg::get<bg::max_corner, 0>(query.boundingbox), bg::get<bg::max_corner, 1>(query.boundingbox),
                      bg::get<bg::max_corner, 2>(query.boundingbox));
    tf2::Vector3 vmaxtransformed = transform * vmax;
    bg::set<bg::max_corner, 0>(queryTransformed.boundingbox, vmaxtransformed.getX());
    bg::set<bg::max_corner, 1>(queryTransformed.boundingbox, vmaxtransformed.getY());
    bg::set<bg::max_corner, 2>(queryTransformed.boundingbox, vmaxtransformed.getZ());

    return queryTransformed;
  }
  else
  {
    return query;
  }
}

std::vector<std::string> CoreTf::getFrames()
{
  BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::trace) << m_tfbuffer.allFramesAsString();

  return std::vector<std::string>{ m_tfbuffer.allFramesAsYAML() };
}

void CoreTf::addToTfBuffer(geometry_msgs::TransformStamped transform)
{
  m_tfbuffer.setTransform(transform, "fromHDF5");
}

} /* namespace seerep_core */
