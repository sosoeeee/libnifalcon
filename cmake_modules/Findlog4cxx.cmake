# - Try to find log4cxx
# Once done this will define
#
#  LOG4CXX_1_FOUND - system has log4cxx
#  LOG4CXX_1_INCLUDE_DIRS - the log4cxx include directory
#  LOG4CXX_1_LIBRARIES - Link these to use log4cxx
#  LOG4CXX_1_DEFINITIONS - Compiler switches required for using log4cxx
#
#  Adapted from cmake-modules Google Code project
#
#  Copyright (c) 2006 Andreas Schneider <mail@cynapses.org>
#
#  (Changes for log4cxx) Copyright (c) 2008 Kyle Machulis <kyle@nonpolynomial.com>
#
# Redistribution and use is allowed according to the terms of the New BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#


if (LOG4CXX_1_LIBRARIES AND LOG4CXX_1_INCLUDE_DIRS)
  # in cache already
  set(LOG4CXX_FOUND TRUE)
else (LOG4CXX_1_LIBRARIES AND LOG4CXX_1_INCLUDE_DIRS)
  find_path(LOG4CXX_1_INCLUDE_DIR
    NAMES
	log4cxx/log4cxx.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
	PATH_SUFFIXES
	  log4cxx
  )

  find_library(LOG4CXX_1_LIBRARY
    NAMES
      log4cxx
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(LOG4CXX_1_INCLUDE_DIRS
    ${LOG4CXX_1_INCLUDE_DIR}
  )
  set(LOG4CXX_1_LIBRARIES
    ${LOG4CXX_1_LIBRARY}
)

  if (LOG4CXX_1_INCLUDE_DIRS AND LOG4CXX_1_LIBRARIES)
     set(LOG4CXX_1_FOUND TRUE)
  endif (LOG4CXX_1_INCLUDE_DIRS AND LOG4CXX_1_LIBRARIES)

  if (LOG4CXX_1_FOUND)
    if (NOT log4cxx_1_FIND_QUIETLY)
      message(STATUS "Found log4cxx:")
	  message(STATUS " - Includes: ${LOG4CXX_1_INCLUDE_DIRS}")
	  message(STATUS " - Libraries: ${LOG4CXX_1_LIBRARIES}")
    endif (NOT log4cxx_1_FIND_QUIETLY)
  else (LOG4CXX_1_FOUND)
    if (log4cxx_1_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find log4cxx")
    endif (log4cxx_1_FIND_REQUIRED)
  endif (LOG4CXX_1_FOUND)

  # show the LOG4CXX_1_INCLUDE_DIRS and LOG4CXX_1_LIBRARIES variables only in the advanced view
  mark_as_advanced(LOG4CXX_1_INCLUDE_DIRS LOG4CXX_1_LIBRARIES)

endif (LOG4CXX_1_LIBRARIES AND LOG4CXX_1_INCLUDE_DIRS)