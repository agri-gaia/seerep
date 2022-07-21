#include "seerep-hdf5-core/hdf5-core-general.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_core
{
Hdf5CoreGeneral::Hdf5CoreGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : m_file(file), m_write_mtx(write_mtx)
{
}

std::vector<std::string> Hdf5CoreGeneral::getGroupDatasets(const std::string& id)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::vector<std::string> rootObjects = m_file->listObjectNames();

  if (id.empty())
  {
    return rootObjects;
  }
  else
  {
    // check if rootObjects contains the group id
    if (std::find(rootObjects.begin(), rootObjects.end(), id) != rootObjects.end())
    {
      return m_file->getGroup(id).listObjectNames();
    }
    else
    {
      return std::vector<std::string>();
    }
  }
}

void Hdf5CoreGeneral::writeProjectname(const std::string& projectname)
{
  const std::scoped_lock lock(*m_write_mtx);

  if (!m_file->hasAttribute(PROJECTNAME))
  {
    m_file->createAttribute<std::string>(PROJECTNAME, projectname);
  }
  else
  {
    m_file->getAttribute(PROJECTNAME).write(projectname);
  }
  m_file->flush();
}

std::string Hdf5CoreGeneral::readProjectname()
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string projectname;
  if (m_file->hasAttribute(PROJECTNAME))
  {
    m_file->getAttribute(PROJECTNAME).read(projectname);
  }
  return projectname;
}

void Hdf5CoreGeneral::writeProjectFrameId(const std::string& frameId)
{
  const std::scoped_lock lock(*m_write_mtx);

  if (!m_file->hasAttribute(PROJECTFRAMEID))
  {
    m_file->createAttribute<std::string>(PROJECTFRAMEID, frameId);
  }
  else
  {
    m_file->getAttribute(PROJECTFRAMEID).write(frameId);
  }
  m_file->flush();
}

std::string Hdf5CoreGeneral::readProjectFrameId()
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string frameId;
  if (m_file->hasAttribute(PROJECTFRAMEID))
  {
    m_file->getAttribute(PROJECTFRAMEID).read(frameId);
  }
  return frameId;
}

}  // namespace seerep_hdf5_core
