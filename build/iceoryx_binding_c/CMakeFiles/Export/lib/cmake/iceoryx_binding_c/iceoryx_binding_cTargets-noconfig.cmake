#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "iceoryx_binding_c::iceoryx_binding_c" for configuration ""
set_property(TARGET iceoryx_binding_c::iceoryx_binding_c APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(iceoryx_binding_c::iceoryx_binding_c PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "iceoryx_posh::iceoryx_posh;iceoryx_hoofs::iceoryx_hoofs"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libiceoryx_binding_c.so"
  IMPORTED_SONAME_NOCONFIG "libiceoryx_binding_c.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS iceoryx_binding_c::iceoryx_binding_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_iceoryx_binding_c::iceoryx_binding_c "${_IMPORT_PREFIX}/lib/libiceoryx_binding_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
