# cmake-lint: disable=C0301

# * Searches for an installation of the HighFive HDF5 library
#
# Defines:
#
# HighFive_FOUND         True if HighFive was found, else false
# HighFive_INCLUDE_DIRS  The directories containing the header files
# HighFive_LIBRARIES     Libraries to link (HighFive is header-only but depends
# on libhdf5 which is not) HighFive_LIBRARY_DIRS  Directories for libraries.
# Pass this to the linker as well. HighFive_CFLAGS        Extra compiler flags
# HighFive_IS_PARALLEL   Whether underlying HDF5 implementation supports
# parallel I/O
#
# To specify an additional directory to search, set HighFive_ROOT.
#
# Author: Siddhartha Chaudhuri, 2020
#

set(HighFive_FOUND FALSE)

# Look for the HighFive header, first in the user-specified location and then in
# the system locations
set(HighFive_INCLUDE_DOC
    "The directory containing the HighFive include file highfive/H5Object.hpp"
)
find_path(
  HighFive_INCLUDE_DIRS
  NAMES H5Object.hpp
  PATHS ${HighFive_ROOT}
  PATH_SUFFIXES
    include/highfive
    include/HighFive
    highfive
    HighFive
  DOC ${HighFive_INCLUDE_DOC}
  NO_DEFAULT_PATH
)
if(NOT HighFive_INCLUDE_DIRS) # now look in system locations
  find_path(
    HighFive_INCLUDE_DIRS
    NAMES H5Object.hpp
    PATHS ${HighFive_ROOT}
    PATH_SUFFIXES
      include/highfive
      include/HighFive
      highfive
      HighFive
    DOC ${HighFive_INCLUDE_DOC}
  )
endif(NOT HighFive_INCLUDE_DIRS)

if(HighFive_INCLUDE_DIRS)

  # Work around a FindHDF5 bug (?) that causes system paths to be ignored if
  # HDF5_ROOT is specified
  set(HDF5_ROOT ${HighFive_ROOT})
  find_package(HDF5 QUIET)
  if(NOT HDF5_FOUND)
    set(HDF5_ROOT)
    find_package(HDF5 REQUIRED)
  endif()

  if(HDF5_FOUND)
    set(HighFive_INCLUDE_DIRS ${HDF5_INCLUDE_DIRS} ${HighFive_INCLUDE_DIRS})
    set(HighFive_LIBRARIES ${HighFive_LIBRARIES} ${HDF5_LIBRARIES})
    set(HighFive_LIBRARY_DIRS ${HighFive_LIBRARY_DIRS} ${HDF5_LIBRARY_DIRS})
    set(HighFive_CFLAGS "${HighFive_CFLAGS} ${HDF5_DEFINITIONS}")
    set(HighFive_IS_PARALLEL ${HDF5_IS_PARALLEL})

    set(HighFive_FOUND TRUE)
  endif(HDF5_FOUND)

endif(HighFive_INCLUDE_DIRS)

if(HighFive_FOUND)
  if(NOT HighFive_FIND_QUIETLY)
    message(
      STATUS
        "Found HighFive: headers at ${HighFive_INCLUDE_DIRS}, libraries at ${HighFive_LIBRARIES}"
    )
  endif(NOT HighFive_FIND_QUIETLY)
else(HighFive_FOUND)
  if(HighFive_FIND_REQUIRED)
    message(FATAL_ERROR "HighFive not found")
  endif(HighFive_FIND_REQUIRED)
endif(HighFive_FOUND)
