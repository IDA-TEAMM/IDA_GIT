# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target rosidl_generator_c::rosidl_generator_c
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${rosidl_generator_c_TARGETS}.
if(rosidl_generator_c_TARGETS AND NOT TARGET rosidl_generator_c::rosidl_generator_c)
  add_library(rosidl_generator_c::rosidl_generator_c INTERFACE IMPORTED)
  set_target_properties(rosidl_generator_c::rosidl_generator_c PROPERTIES
    INTERFACE_LINK_LIBRARIES "${rosidl_generator_c_TARGETS}")
endif()
