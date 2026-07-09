// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosidl_generator_cpp:srv/Empty.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_GENERATOR_CPP__SRV__DETAIL__EMPTY__TRAITS_HPP_
#define ROSIDL_GENERATOR_CPP__SRV__DETAIL__EMPTY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosidl_generator_cpp/srv/detail/empty__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosidl_generator_cpp
{

namespace srv
{

inline void to_flow_style_yaml(
  const Empty_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Empty_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Empty_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rosidl_generator_cpp

namespace rosidl_generator_traits
{

[[deprecated("use rosidl_generator_cpp::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rosidl_generator_cpp::srv::Empty_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosidl_generator_cpp::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosidl_generator_cpp::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosidl_generator_cpp::srv::Empty_Request & msg)
{
  return rosidl_generator_cpp::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosidl_generator_cpp::srv::Empty_Request>()
{
  return "rosidl_generator_cpp::srv::Empty_Request";
}

template<>
inline const char * name<rosidl_generator_cpp::srv::Empty_Request>()
{
  return "rosidl_generator_cpp/srv/Empty_Request";
}

template<>
struct has_fixed_size<rosidl_generator_cpp::srv::Empty_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rosidl_generator_cpp::srv::Empty_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rosidl_generator_cpp::srv::Empty_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_cpp
{

namespace srv
{

inline void to_flow_style_yaml(
  const Empty_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Empty_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Empty_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rosidl_generator_cpp

namespace rosidl_generator_traits
{

[[deprecated("use rosidl_generator_cpp::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rosidl_generator_cpp::srv::Empty_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosidl_generator_cpp::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosidl_generator_cpp::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosidl_generator_cpp::srv::Empty_Response & msg)
{
  return rosidl_generator_cpp::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosidl_generator_cpp::srv::Empty_Response>()
{
  return "rosidl_generator_cpp::srv::Empty_Response";
}

template<>
inline const char * name<rosidl_generator_cpp::srv::Empty_Response>()
{
  return "rosidl_generator_cpp/srv/Empty_Response";
}

template<>
struct has_fixed_size<rosidl_generator_cpp::srv::Empty_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rosidl_generator_cpp::srv::Empty_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rosidl_generator_cpp::srv::Empty_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rosidl_generator_cpp::srv::Empty>()
{
  return "rosidl_generator_cpp::srv::Empty";
}

template<>
inline const char * name<rosidl_generator_cpp::srv::Empty>()
{
  return "rosidl_generator_cpp/srv/Empty";
}

template<>
struct has_fixed_size<rosidl_generator_cpp::srv::Empty>
  : std::integral_constant<
    bool,
    has_fixed_size<rosidl_generator_cpp::srv::Empty_Request>::value &&
    has_fixed_size<rosidl_generator_cpp::srv::Empty_Response>::value
  >
{
};

template<>
struct has_bounded_size<rosidl_generator_cpp::srv::Empty>
  : std::integral_constant<
    bool,
    has_bounded_size<rosidl_generator_cpp::srv::Empty_Request>::value &&
    has_bounded_size<rosidl_generator_cpp::srv::Empty_Response>::value
  >
{
};

template<>
struct is_service<rosidl_generator_cpp::srv::Empty>
  : std::true_type
{
};

template<>
struct is_service_request<rosidl_generator_cpp::srv::Empty_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rosidl_generator_cpp::srv::Empty_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROSIDL_GENERATOR_CPP__SRV__DETAIL__EMPTY__TRAITS_HPP_
