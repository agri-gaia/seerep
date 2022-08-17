namespace seerep_core_fb
{
template <class C>
void CoreFbGeneral::getAllFileAccessorFromCore(
    std::shared_ptr<seerep_core::Core>& seerepCore,
    std::unordered_map<boost::uuids::uuid, std::shared_ptr<C>, boost::hash<boost::uuids::uuid>>& hdf5IoMap)
{
  for (seerep_core_msgs::ProjectInfo projectInfo : seerepCore->getProjects())
  {
    CoreFbGeneral::getFileAccessorFromCore(projectInfo.uuid, seerepCore, hdf5IoMap);
  }
}

template <class C>
void CoreFbGeneral::getFileAccessorFromCore(
    const std::string& project, std::shared_ptr<seerep_core::Core>& seerepCore,
    std::unordered_map<boost::uuids::uuid, std::shared_ptr<C>, boost::hash<boost::uuids::uuid>>& hdf5IoMap)
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid projectUuid = gen(project);
  getFileAccessorFromCore(projectUuid, seerepCore, hdf5IoMap);
}

template <class C>
void CoreFbGeneral::getFileAccessorFromCore(
    const boost::uuids::uuid& project, std::shared_ptr<seerep_core::Core>& seerepCore,
    std::unordered_map<boost::uuids::uuid, std::shared_ptr<C>, boost::hash<boost::uuids::uuid>>& hdf5IoMap)
{
  auto hdf5file = seerepCore->getHdf5File(project);
  auto hdf5fileMutex = seerepCore->getHdf5FileMutex(project);
  auto datatypeIo = std::make_shared<C>(hdf5file, hdf5fileMutex);
  hdf5IoMap.insert(std::make_pair(project, datatypeIo));
}

template <class C>
std::shared_ptr<C> CoreFbGeneral::getHdf5(
    const std::string& project, std::shared_ptr<seerep_core::Core>& seerepCore,
    std::unordered_map<boost::uuids::uuid, std::shared_ptr<C>, boost::hash<boost::uuids::uuid>>& hdf5IoMap)
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid projectUuid = gen(project);
  return getHdf5(projectUuid, seerepCore, hdf5IoMap);
}

template <class C>
std::shared_ptr<C> CoreFbGeneral::getHdf5(
    const boost::uuids::uuid& project, std::shared_ptr<seerep_core::Core>& seerepCore,
    std::unordered_map<boost::uuids::uuid, std::shared_ptr<C>, boost::hash<boost::uuids::uuid>>& hdf5IoMap)
{
  // find the project based on its uuid
  auto hdf5io = hdf5IoMap.find(project);
  // if project was found add tf
  if (hdf5io != hdf5IoMap.end())
  {
    return hdf5io->second;
  }
  // if not found ask core
  else
  {
    // this throws an exeption if core has no project with the uuid
    getFileAccessorFromCore(project, seerepCore, hdf5IoMap);
    // if getFileAccessorFromCore didn't throw an error, find project and return pointer
    return hdf5IoMap.find(project)->second;
  };
}

}  // namespace seerep_core_fb
