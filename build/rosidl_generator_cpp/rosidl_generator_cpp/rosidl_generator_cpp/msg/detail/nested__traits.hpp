// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosidl_generator_cpp:msg/Nested.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_GENERATOR_CPP__MSG__DETAIL__NESTED__TRAITS_HPP_
#define ROSIDL_GENERATOR_CPP__MSG__DETAIL__NESTED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosidl_generator_cpp/msg/detail/nested__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'basic_types_value'
#include "rosidl_generator_cpp/msg/detail/basic_types__traits.hpp"

namespace rosidl_generator_cpp
{

namespace msg
{

inline void to_flow_style_yaml(
  const Nested & msg,
  std::ostream & out)
{
  out << "{";
  // member: basic_types_value
  {
    out << "basic_types_value: ";
    to_flow_style_yaml(msg.basic_types_value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Nested & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: basic_types_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "basic_types_value:\n";
    to_block_style_yaml(msg.basic_types_value, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Nested & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace rosidl_generator_cpp

namespace rosidl_generator_traits
{

[[deprecated("use rosidl_generator_cpp::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rosidl_generator_cpp::msg::Nested & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosidl_generator_cpp::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosidl_generator_cpp::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosidl_generator_cpp::msg::Nested & msg)
{
  return rosidl_generator_cpp::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosidl_generator_cpp::msg::Nested>()
{
  return "rosidl_generator_cpp::msg::Nested";
}

template<>
inline const char * name<rosidl_generator_cpp::msg::Nested>()
{
  return "rosidl_generator_cpp/msg/Nested";
}

template<>
struct has_fixed_size<rosidl_generator_cpp::msg::Nested>
  : std::integral_constant<bool, has_fixed_size<rosidl_generator_cpp::msg::BasicTypes>::value> {};

template<>
struct has_bounded_size<rosidl_generator_cpp::msg::Nested>
  : std::integral_constant<bool, has_bounded_size<rosidl_generator_cpp::msg::BasicTypes>::value> {};

template<>
struct is_message<rosidl_generator_cpp::msg::Nested>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSIDL_GENERATOR_CPP__MSG__DETAIL__NESTED__TRAITS_HPP_
