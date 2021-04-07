# - Searches for an installation of the HighFive HDF5 library
#
# Defines:
#
#   HighFive_FOUND         True if HighFive was found, else false
#   HighFive_INCLUDE_DIRS  The directories containing the header files
#   HighFive_LIBRARIES     Libraries to link (HighFive is header-only but depends on libhdf5 which is not)
#   HighFive_LIBRARY_DIRS  Directories for libraries. Pass this to the linker as well.
#   HighFive_CFLAGS        Extra compiler flags
#   HighFive_IS_PARALLEL   Whether underlying HDF5 implementation supports parallel I/O
#
# To specify an additional directory to search, set HighFive_ROOT.
#
# Author: Siddhartha Chaudhuri, 2020
#

SET(HighFive_FOUND FALSE)

# Look for the HighFive header, first in the user-specified location and then in the system locations
SET(HighFive_INCLUDE_DOC "The directory containing the HighFive include file highfive/H5Object.hpp")
FIND_PATH(HighFive_INCLUDE_DIRS NAMES H5Object.hpp
  PATHS ${HighFive_ROOT}
  PATH_SUFFIXES include/highfive include/HighFive highfive HighFive
  DOC ${HighFive_INCLUDE_DOC} NO_DEFAULT_PATH)
IF(NOT HighFive_INCLUDE_DIRS)  # now look in system locations
  FIND_PATH(HighFive_INCLUDE_DIRS NAMES H5Object.hpp
    PATHS ${HighFive_ROOT}
    PATH_SUFFIXES include/highfive include/HighFive highfive HighFive
    DOC ${HighFive_INCLUDE_DOC})
ENDIF(NOT HighFive_INCLUDE_DIRS)

IF(HighFive_INCLUDE_DIRS)

  # Work around a FindHDF5 bug (?) that causes system paths to be ignored if HDF5_ROOT is specified
  SET(HDF5_ROOT ${HighFive_ROOT})
  FIND_PACKAGE(HDF5 QUIET)
  IF(NOT HDF5_FOUND)
    SET(HDF5_ROOT )
    FIND_PACKAGE(HDF5 REQUIRED)
  ENDIF()

  IF(HDF5_FOUND)
    SET(HighFive_INCLUDE_DIRS ${HDF5_INCLUDE_DIRS} ${HighFive_INCLUDE_DIRS})
    SET(HighFive_LIBRARIES ${HighFive_LIBRARIES} ${HDF5_LIBRARIES})
    SET(HighFive_LIBRARY_DIRS ${HighFive_LIBRARY_DIRS} ${HDF5_LIBRARY_DIRS})
    SET(HighFive_CFLAGS "${HighFive_CFLAGS} ${HDF5_DEFINITIONS}")
    SET(HighFive_IS_PARALLEL ${HDF5_IS_PARALLEL})

    SET(HighFive_FOUND TRUE)
  ENDIF(HDF5_FOUND)

ENDIF(HighFive_INCLUDE_DIRS)

IF(HighFive_FOUND)
  IF(NOT HighFive_FIND_QUIETLY)
    MESSAGE(STATUS "Found HighFive: headers at ${HighFive_INCLUDE_DIRS}, libraries at ${HighFive_LIBRARIES}")
  ENDIF(NOT HighFive_FIND_QUIETLY)
ELSE(HighFive_FOUND)
  IF(HighFive_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "HighFive not found")
  ENDIF(HighFive_FIND_REQUIRED)
ENDIF(HighFive_FOUND)