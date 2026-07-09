// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosidl_generator_cpp:msg/Constants.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_GENERATOR_CPP__MSG__DETAIL__CONSTANTS__TRAITS_HPP_
#define ROSIDL_GENERATOR_CPP__MSG__DETAIL__CONSTANTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosidl_generator_cpp/msg/detail/constants__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosidl_generator_cpp
{

namespace msg
{

inline void to_flow_style_yaml(
  const Constants & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Constants & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Constants & msg, bool use_flow_style = false)
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
  const rosidl_generator_cpp::msg::Constants & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosidl_generator_cpp::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosidl_generator_cpp::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosidl_generator_cpp::msg::Constants & msg)
{
  return rosidl_generator_cpp::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosidl_generator_cpp::msg::Constants>()
{
  return "rosidl_generator_cpp::msg::Constants";
}

template<>
inline const char * name<rosidl_generator_cpp::msg::Constants>()
{
  return "rosidl_generator_cpp/msg/Constants";
}

template<>
struct has_fixed_size<rosidl_generator_cpp::msg::Constants>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rosidl_generator_cpp::msg::Constants>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rosidl_generator_cpp::msg::Constants>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSIDL_GENERATOR_CPP__MSG__DETAIL__CONSTANTS__TRAITS_HPP_
