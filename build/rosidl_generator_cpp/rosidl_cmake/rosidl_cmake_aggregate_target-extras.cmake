# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target rosidl_generator_cpp::rosidl_generator_cpp
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${rosidl_generator_cpp_TARGETS}.
if(rosidl_generator_cpp_TARGETS AND NOT TARGET rosidl_generator_cpp::rosidl_generator_cpp)
  add_library(rosidl_generator_cpp::rosidl_generator_cpp INTERFACE IMPORTED)
  set_target_properties(rosidl_generator_cpp::rosidl_generator_cpp PROPERTIES
    INTERFACE_LINK_LIBRARIES "${rosidl_generator_cpp_TARGETS}")
endif()
