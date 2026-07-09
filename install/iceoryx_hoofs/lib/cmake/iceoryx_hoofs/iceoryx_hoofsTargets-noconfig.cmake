#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "iceoryx_hoofs::iceoryx_hoofs" for configuration ""
set_property(TARGET iceoryx_hoofs::iceoryx_hoofs APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(iceoryx_hoofs::iceoryx_hoofs PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libiceoryx_hoofs.so"
  IMPORTED_SONAME_NOCONFIG "libiceoryx_hoofs.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS iceoryx_hoofs::iceoryx_hoofs )
list(APPEND _IMPORT_CHECK_FILES_FOR_iceoryx_hoofs::iceoryx_hoofs "${_IMPORT_PREFIX}/lib/libiceoryx_hoofs.so" )

# Import target "iceoryx_hoofs::iceoryx_platform" for configuration ""
set_property(TARGET iceoryx_hoofs::iceoryx_platform APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(iceoryx_hoofs::iceoryx_platform PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libiceoryx_platform.so"
  IMPORTED_SONAME_NOCONFIG "libiceoryx_platform.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS iceoryx_hoofs::iceoryx_platform )
list(APPEND _IMPORT_CHECK_FILES_FOR_iceoryx_hoofs::iceoryx_platform "${_IMPORT_PREFIX}/lib/libiceoryx_platform.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
