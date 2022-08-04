#ifndef SEEREP_CORE_FB_GENERAL_H_
#define SEEREP_CORE_FB_GENERAL_H_

#include <functional>
#include <optional>

#include "core-fb-conversion.h"

// seerep-core
#include <seerep-core/core.h>

// uuid
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // uuid class
#include <boost/uuid/uuid_generators.hpp>  // generators
#include <boost/uuid/uuid_io.hpp>          // streaming operators etc.

// logging
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

namespace seerep_core_fb
{
/**
 * @brief General functions for the other core-fb classes
 */
class CoreFbGeneral
{
public:
  /**
   * @brief extracts the hdf5-io object from the hdf5-io-map for the given project
   * @param project the uuid of the project for which the io-object is needed
   * @param seerepCore pointer to the seerep core which is needed to get the io-objects
   * @param hdf5IoMap the map to store the io-objects
   */
  template <class C>
  static std::shared_ptr<C>
  getHdf5(const boost::uuids::uuid& project, std::shared_ptr<seerep_core::Core>& seerepCore,
          std::unordered_map<boost::uuids::uuid, std::shared_ptr<C>, boost::hash<boost::uuids::uuid>>& hdf5IoMap);

  /**
   * @brief gets the file accessors (the hdf5 file object itself, the mutex, the io object) for the hdf5 file
   * @param project the uuid of the project for which the accessors are needed
   * @param seerepCore pointer to the seerep core which is needed to get the accessors
   * @param hdf5IoMap the map to store the accessors
   */
  template <class C>
  static void getFileAccessorFromCore(
      const boost::uuids::uuid& project, std::shared_ptr<seerep_core::Core>& seerepCore,
      std::unordered_map<boost::uuids::uuid, std::shared_ptr<C>, boost::hash<boost::uuids::uuid>>& hdf5IoMap);
  /**
   * @brief gets the file accessors (the hdf5 file object itself, the mutex, the io object) for all the hdf5 files
   * @param seerepCore pointer to the seerep core which is needed to get the accessors
   * @param hdf5IoMap the map to store the accessors
   */
  template <class C>
  static void getAllFileAccessorFromCore(
      std::shared_ptr<seerep_core::Core>& seerepCore,
      std::unordered_map<boost::uuids::uuid, std::shared_ptr<C>, boost::hash<boost::uuids::uuid>>& hdf5IoMap);

private:
  /** the logger for the logging framework */
  static boost::log::sources::severity_logger<boost::log::trivial::severity_level> m_logger;
};

}  // namespace seerep_core_fb

#include "impl/core-fb-general.hpp"  // NOLINT

#endif  // SEEREP_CORE_FB_GENERAL_H_
