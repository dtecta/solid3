include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

find_path(Qhull_REENTRANT_INCLUDE_DIRS NAMES libqhull_r/qhull_ra.h qhull_r/qhull_ra.h)

mark_as_advanced(Qhull_REENTRANT_INCLUDE_DIRS)

if(BUILD_SHARED_LIBS)
  find_library(Qhull_REENTRANT_LIBRARY_DEBUG NAMES qhull_rd)
  find_library(Qhull_REENTRANT_LIBRARY_RELEASE NAMES qhull_r)
else()
  find_library(Qhull_REENTRANT_LIBRARY_DEBUG NAMES qhullstatic_rd)
  find_library(Qhull_REENTRANT_LIBRARY_RELEASE NAMES qhullstatic_r)
endif()

select_library_configurations(Qhull_REENTRANT)

find_path(Qhull_NON_REENTRANT_INCLUDE_DIRS NAMES libqhull/qhull_a.h qhull/qhull_a.h)

mark_as_advanced(Qhull_NON_REENTRANT_INCLUDE_DIRS)

if(BUILD_SHARED_LIBS)
  find_library(Qhull_NON_REENTRANT_LIBRARY_DEBUG NAMES qhull_d)
  find_library(Qhull_NON_REENTRANT_LIBRARY_RELEASE NAMES qhull)
else()
  find_library(Qhull_NON_REENTRANT_LIBRARY_DEBUG NAMES qhullstatic_d)
  find_library(Qhull_NON_REENTRANT_LIBRARY_RELEASE NAMES qhullstatic)
endif()

select_library_configurations(Qhull_NON_REENTRANT)

if(Qhull_REENTRANT_INCLUDE_DIRS AND Qhull_REENTRANT_LIBRARIES)
  set(Qhull_INCLUDE_DIRS ${Qhull_REENTRANT_INCLUDE_DIRS})
  set(Qhull_LIBRARIES ${Qhull_REENTRANT_LIBRARIES})
  set(QHull_REENTRANT_FOUND TRUE)
elseif(Qhull_NON_REENTRANT_INCLUDE_DIRS AND Qhull_NON_REENTRANT_LIBRARIES)
  set(Qhull_INCLUDE_DIRS ${Qhull_NON_REENTRANT_INCLUDE_DIRS})
  set(Qhull_LIBRARIES ${Qhull_NON_REENTRANT_LIBRARIES})
  set(QHull_NON_REENTRANT_FOUND TRUE)
endif()

find_package_handle_standard_args(
  Qhull
  FOUND_VAR Qhull_FOUND
  REQUIRED_VARS Qhull_INCLUDE_DIRS Qhull_LIBRARIES
)
