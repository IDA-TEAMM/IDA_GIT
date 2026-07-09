// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosidl_generator_cpp:msg/WStrings.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_GENERATOR_CPP__MSG__DETAIL__W_STRINGS__TRAITS_HPP_
#define ROSIDL_GENERATOR_CPP__MSG__DETAIL__W_STRINGS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosidl_generator_cpp/msg/detail/w_strings__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosidl_generator_cpp
{

namespace msg
{

inline void to_flow_style_yaml(
  const WStrings & msg,
  std::ostream & out)
{
  out << "{";
  // member: wstring_value
  {
    out << "wstring_value: ";
    rosidl_generator_traits::value_to_yaml(msg.wstring_value, out);
    out << ", ";
  }

  // member: wstring_value_default1
  {
    out << "wstring_value_default1: ";
    rosidl_generator_traits::value_to_yaml(msg.wstring_value_default1, out);
    out << ", ";
  }

  // member: wstring_value_default2
  {
    out << "wstring_value_default2: ";
    rosidl_generator_traits::value_to_yaml(msg.wstring_value_default2, out);
    out << ", ";
  }

  // member: wstring_value_default3
  {
    out << "wstring_value_default3: ";
    rosidl_generator_traits::value_to_yaml(msg.wstring_value_default3, out);
    out << ", ";
  }

  // member: array_of_wstrings
  {
    if (msg.array_of_wstrings.size() == 0) {
      out << "array_of_wstrings: []";
    } else {
      out << "array_of_wstrings: [";
      size_t pending_items = msg.array_of_wstrings.size();
      for (auto item : msg.array_of_wstrings) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: bounded_sequence_of_wstrings
  {
    if (msg.bounded_sequence_of_wstrings.size() == 0) {
      out << "bounded_sequence_of_wstrings: []";
    } else {
      out << "bounded_sequence_of_wstrings: [";
      size_t pending_items = msg.bounded_sequence_of_wstrings.size();
      for (auto item : msg.bounded_sequence_of_wstrings) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: unbounded_sequence_of_wstrings
  {
    if (msg.unbounded_sequence_of_wstrings.size() == 0) {
      out << "unbounded_sequence_of_wstrings: []";
    } else {
      out << "unbounded_sequence_of_wstrings: [";
      size_t pending_items = msg.unbounded_sequence_of_wstrings.size();
      for (auto item : msg.unbounded_sequence_of_wstrings) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WStrings & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: wstring_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wstring_value: ";
    rosidl_generator_traits::value_to_yaml(msg.wstring_value, out);
    out << "\n";
  }

  // member: wstring_value_default1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wstring_value_default1: ";
    rosidl_generator_traits::value_to_yaml(msg.wstring_value_default1, out);
    out << "\n";
  }

  // member: wstring_value_default2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wstring_value_default2: ";
    rosidl_generator_traits::value_to_yaml(msg.wstring_value_default2, out);
    out << "\n";
  }

  // member: wstring_value_default3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wstring_value_default3: ";
    rosidl_generator_traits::value_to_yaml(msg.wstring_value_default3, out);
    out << "\n";
  }

  // member: array_of_wstrings
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.array_of_wstrings.size() == 0) {
      out << "array_of_wstrings: []\n";
    } else {
      out << "array_of_wstrings:\n";
      for (auto item : msg.array_of_wstrings) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: bounded_sequence_of_wstrings
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bounded_sequence_of_wstrings.size() == 0) {
      out << "bounded_sequence_of_wstrings: []\n";
    } else {
      out << "bounded_sequence_of_wstrings:\n";
      for (auto item : msg.bounded_sequence_of_wstrings) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: unbounded_sequence_of_wstrings
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.unbounded_sequence_of_wstrings.size() == 0) {
      out << "unbounded_sequence_of_wstrings: []\n";
    } else {
      out << "unbounded_sequence_of_wstrings:\n";
      for (auto item : msg.unbounded_sequence_of_wstrings) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WStrings & msg, bool use_flow_style = false)
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
  const rosidl_generator_cpp::msg::WStrings & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosidl_generator_cpp::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosidl_generator_cpp::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosidl_generator_cpp::msg::WStrings & msg)
{
  return rosidl_generator_cpp::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosidl_generator_cpp::msg::WStrings>()
{
  return "rosidl_generator_cpp::msg::WStrings";
}

template<>
inline const char * name<rosidl_generator_cpp::msg::WStrings>()
{
  return "rosidl_generator_cpp/msg/WStrings";
}

template<>
struct has_fixed_size<rosidl_generator_cpp::msg::WStrings>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rosidl_generator_cpp::msg::WStrings>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rosidl_generator_cpp::msg::WStrings>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSIDL_GENERATOR_CPP__MSG__DETAIL__W_STRINGS__TRAITS_HPP_
