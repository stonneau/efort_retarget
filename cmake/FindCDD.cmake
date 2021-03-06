# Base Io build system
# Written by Steve Tonneau steveteonneau@hotmail.fr
#
# Find PQP
FIND_PATH(CDD_INCLUDE_DIR libcdd/cdd.h
    /usr/include
    /usr/local/include
)

SET(CDD_NAMES ${CDD_NAMES} cdd libcdd)
FIND_LIBRARY(CDD_LIBRARY NAMES ${CDD_NAMES} PATH)

IF(CDD_LIBRARY)
    MESSAGE(STATUS "Found CDD library: ${CDD_LIBRARY}")
ELSE(CDD_LIBRARY)
    MESSAGE(STATUS "Coulddn't find CDD library: ${CDD_LIBRARY}")
ENDIF(CDD_LIBRARY)

IF(CDD_INCLUDE_DIR AND CDD_LIBRARY)
SET(CDD_FOUND TRUE CACHE STRING "Whether CDD was found or not")
ENDIF(CDD_INCLUDE_DIR AND CDD_LIBRARY)

IF(CDD_FOUND)
    SET(CMAKE_C_FLAGS "-DdSINGLE")
IF(NOT CDD_FIND_QUIETLY)
MESSAGE(STATUS "Found CDD: ${CDD_LIBRARY}")
ENDIF (NOT CDD_FIND_QUIETLY)
ELSE(CDD_FOUND)
IF(CDD_FIND_REQUIRED)
MESSAGE(FATAL_ERROR "Could not find CDD")
ENDIF(CDD_FIND_REQUIRED)
ENDIF(CDD_FOUND)
