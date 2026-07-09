// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosidl_generator_cpp:msg/MultiNested.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_GENERATOR_CPP__MSG__DETAIL__MULTI_NESTED__TRAITS_HPP_
#define ROSIDL_GENERATOR_CPP__MSG__DETAIL__MULTI_NESTED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosidl_generator_cpp/msg/detail/multi_nested__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'array_of_arrays'
// Member 'bounded_sequence_of_arrays'
// Member 'unbounded_sequence_of_arrays'
#include "rosidl_generator_cpp/msg/detail/arrays__traits.hpp"
// Member 'array_of_bounded_sequences'
// Member 'bounded_sequence_of_bounded_sequences'
// Member 'unbounded_sequence_of_bounded_sequences'
#include "rosidl_generator_cpp/msg/detail/bounded_sequences__traits.hpp"
// Member 'array_of_unbounded_sequences'
// Member 'bounded_sequence_of_unbounded_sequences'
// Member 'unbounded_sequence_of_unbounded_sequences'
#include "rosidl_generator_cpp/msg/detail/unbounded_sequences__traits.hpp"

namespace rosidl_generator_cpp
{

namespace msg
{

inline void to_flow_style_yaml(
  const MultiNested & msg,
  std::ostream & out)
{
  out << "{";
  // member: array_of_arrays
  {
    if (msg.array_of_arrays.size() == 0) {
      out << "array_of_arrays: []";
    } else {
      out << "array_of_arrays: [";
      size_t pending_items = msg.array_of_arrays.size();
      for (auto item : msg.array_of_arrays) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: array_of_bounded_sequences
  {
    if (msg.array_of_bounded_sequences.size() == 0) {
      out << "array_of_bounded_sequences: []";
    } else {
      out << "array_of_bounded_sequences: [";
      size_t pending_items = msg.array_of_bounded_sequences.size();
      for (auto item : msg.array_of_bounded_sequences) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: array_of_unbounded_sequences
  {
    if (msg.array_of_unbounded_sequences.size() == 0) {
      out << "array_of_unbounded_sequences: []";
    } else {
      out << "array_of_unbounded_sequences: [";
      size_t pending_items = msg.array_of_unbounded_sequences.size();
      for (auto item : msg.array_of_unbounded_sequences) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: bounded_sequence_of_arrays
  {
    if (msg.bounded_sequence_of_arrays.size() == 0) {
      out << "bounded_sequence_of_arrays: []";
    } else {
      out << "bounded_sequence_of_arrays: [";
      size_t pending_items = msg.bounded_sequence_of_arrays.size();
      for (auto item : msg.bounded_sequence_of_arrays) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: bounded_sequence_of_bounded_sequences
  {
    if (msg.bounded_sequence_of_bounded_sequences.size() == 0) {
      out << "bounded_sequence_of_bounded_sequences: []";
    } else {
      out << "bounded_sequence_of_bounded_sequences: [";
      size_t pending_items = msg.bounded_sequence_of_bounded_sequences.size();
      for (auto item : msg.bounded_sequence_of_bounded_sequences) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: bounded_sequence_of_unbounded_sequences
  {
    if (msg.bounded_sequence_of_unbounded_sequences.size() == 0) {
      out << "bounded_sequence_of_unbounded_sequences: []";
    } else {
      out << "bounded_sequence_of_unbounded_sequences: [";
      size_t pending_items = msg.bounded_sequence_of_unbounded_sequences.size();
      for (auto item : msg.bounded_sequence_of_unbounded_sequences) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: unbounded_sequence_of_arrays
  {
    if (msg.unbounded_sequence_of_arrays.size() == 0) {
      out << "unbounded_sequence_of_arrays: []";
    } else {
      out << "unbounded_sequence_of_arrays: [";
      size_t pending_items = msg.unbounded_sequence_of_arrays.size();
      for (auto item : msg.unbounded_sequence_of_arrays) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: unbounded_sequence_of_bounded_sequences
  {
    if (msg.unbounded_sequence_of_bounded_sequences.size() == 0) {
      out << "unbounded_sequence_of_bounded_sequences: []";
    } else {
      out << "unbounded_sequence_of_bounded_sequences: [";
      size_t pending_items = msg.unbounded_sequence_of_bounded_sequences.size();
      for (auto item : msg.unbounded_sequence_of_bounded_sequences) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: unbounded_sequence_of_unbounded_sequences
  {
    if (msg.unbounded_sequence_of_unbounded_sequences.size() == 0) {
      out << "unbounded_sequence_of_unbounded_sequences: []";
    } else {
      out << "unbounded_sequence_of_unbounded_sequences: [";
      size_t pending_items = msg.unbounded_sequence_of_unbounded_sequences.size();
      for (auto item : msg.unbounded_sequence_of_unbounded_sequences) {
        to_flow_style_yaml(item, out);
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
  const MultiNested & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: array_of_arrays
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.array_of_arrays.size() == 0) {
      out << "array_of_arrays: []\n";
    } else {
      out << "array_of_arrays:\n";
      for (auto item : msg.array_of_arrays) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: array_of_bounded_sequences
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.array_of_bounded_sequences.size() == 0) {
      out << "array_of_bounded_sequences: []\n";
    } else {
      out << "array_of_bounded_sequences:\n";
      for (auto item : msg.array_of_bounded_sequences) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: array_of_unbounded_sequences
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.array_of_unbounded_sequences.size() == 0) {
      out << "array_of_unbounded_sequences: []\n";
    } else {
      out << "array_of_unbounded_sequences:\n";
      for (auto item : msg.array_of_unbounded_sequences) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: bounded_sequence_of_arrays
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bounded_sequence_of_arrays.size() == 0) {
      out << "bounded_sequence_of_arrays: []\n";
    } else {
      out << "bounded_sequence_of_arrays:\n";
      for (auto item : msg.bounded_sequence_of_arrays) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: bounded_sequence_of_bounded_sequences
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bounded_sequence_of_bounded_sequences.size() == 0) {
      out << "bounded_sequence_of_bounded_sequences: []\n";
    } else {
      out << "bounded_sequence_of_bounded_sequences:\n";
      for (auto item : msg.bounded_sequence_of_bounded_sequences) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: bounded_sequence_of_unbounded_sequences
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bounded_sequence_of_unbounded_sequences.size() == 0) {
      out << "bounded_sequence_of_unbounded_sequences: []\n";
    } else {
      out << "bounded_sequence_of_unbounded_sequences:\n";
      for (auto item : msg.bounded_sequence_of_unbounded_sequences) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: unbounded_sequence_of_arrays
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.unbounded_sequence_of_arrays.size() == 0) {
      out << "unbounded_sequence_of_arrays: []\n";
    } else {
      out << "unbounded_sequence_of_arrays:\n";
      for (auto item : msg.unbounded_sequence_of_arrays) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: unbounded_sequence_of_bounded_sequences
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.unbounded_sequence_of_bounded_sequences.size() == 0) {
      out << "unbounded_sequence_of_bounded_sequences: []\n";
    } else {
      out << "unbounded_sequence_of_bounded_sequences:\n";
      for (auto item : msg.unbounded_sequence_of_bounded_sequences) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: unbounded_sequence_of_unbounded_sequences
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.unbounded_sequence_of_unbounded_sequences.size() == 0) {
      out << "unbounded_sequence_of_unbounded_sequences: []\n";
    } else {
      out << "unbounded_sequence_of_unbounded_sequences:\n";
      for (auto item : msg.unbounded_sequence_of_unbounded_sequences) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MultiNested & msg, bool use_flow_style = false)
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
  const rosidl_generator_cpp::msg::MultiNested & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosidl_generator_cpp::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosidl_generator_cpp::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosidl_generator_cpp::msg::MultiNested & msg)
{
  return rosidl_generator_cpp::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosidl_generator_cpp::msg::MultiNested>()
{
  return "rosidl_generator_cpp::msg::MultiNested";
}

template<>
inline const char * name<rosidl_generator_cpp::msg::MultiNested>()
{
  return "rosidl_generator_cpp/msg/MultiNested";
}

template<>
struct has_fixed_size<rosidl_generator_cpp::msg::MultiNested>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rosidl_generator_cpp::msg::MultiNested>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rosidl_generator_cpp::msg::MultiNested>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSIDL_GENERATOR_CPP__MSG__DETAIL__MULTI_NESTED__TRAITS_HPP_
