#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rcl_logging_noop::rcl_logging_noop" for configuration ""
set_property(TARGET rcl_logging_noop::rcl_logging_noop APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rcl_logging_noop::rcl_logging_noop PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "rcutils::rcutils"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librcl_logging_noop.so"
  IMPORTED_SONAME_NOCONFIG "librcl_logging_noop.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rcl_logging_noop::rcl_logging_noop )
list(APPEND _IMPORT_CHECK_FILES_FOR_rcl_logging_noop::rcl_logging_noop "${_IMPORT_PREFIX}/lib/librcl_logging_noop.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
