// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosidl_generator_cpp:msg/WStrings.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_GENERATOR_CPP__MSG__DETAIL__W_STRINGS__BUILDER_HPP_
#define ROSIDL_GENERATOR_CPP__MSG__DETAIL__W_STRINGS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosidl_generator_cpp/msg/detail/w_strings__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosidl_generator_cpp
{

namespace msg
{

namespace builder
{

class Init_WStrings_unbounded_sequence_of_wstrings
{
public:
  explicit Init_WStrings_unbounded_sequence_of_wstrings(::rosidl_generator_cpp::msg::WStrings & msg)
  : msg_(msg)
  {}
  ::rosidl_generator_cpp::msg::WStrings unbounded_sequence_of_wstrings(::rosidl_generator_cpp::msg::WStrings::_unbounded_sequence_of_wstrings_type arg)
  {
    msg_.unbounded_sequence_of_wstrings = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::WStrings msg_;
};

class Init_WStrings_bounded_sequence_of_wstrings
{
public:
  explicit Init_WStrings_bounded_sequence_of_wstrings(::rosidl_generator_cpp::msg::WStrings & msg)
  : msg_(msg)
  {}
  Init_WStrings_unbounded_sequence_of_wstrings bounded_sequence_of_wstrings(::rosidl_generator_cpp::msg::WStrings::_bounded_sequence_of_wstrings_type arg)
  {
    msg_.bounded_sequence_of_wstrings = std::move(arg);
    return Init_WStrings_unbounded_sequence_of_wstrings(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::WStrings msg_;
};

class Init_WStrings_array_of_wstrings
{
public:
  explicit Init_WStrings_array_of_wstrings(::rosidl_generator_cpp::msg::WStrings & msg)
  : msg_(msg)
  {}
  Init_WStrings_bounded_sequence_of_wstrings array_of_wstrings(::rosidl_generator_cpp::msg::WStrings::_array_of_wstrings_type arg)
  {
    msg_.array_of_wstrings = std::move(arg);
    return Init_WStrings_bounded_sequence_of_wstrings(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::WStrings msg_;
};

class Init_WStrings_wstring_value_default3
{
public:
  explicit Init_WStrings_wstring_value_default3(::rosidl_generator_cpp::msg::WStrings & msg)
  : msg_(msg)
  {}
  Init_WStrings_array_of_wstrings wstring_value_default3(::rosidl_generator_cpp::msg::WStrings::_wstring_value_default3_type arg)
  {
    msg_.wstring_value_default3 = std::move(arg);
    return Init_WStrings_array_of_wstrings(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::WStrings msg_;
};

class Init_WStrings_wstring_value_default2
{
public:
  explicit Init_WStrings_wstring_value_default2(::rosidl_generator_cpp::msg::WStrings & msg)
  : msg_(msg)
  {}
  Init_WStrings_wstring_value_default3 wstring_value_default2(::rosidl_generator_cpp::msg::WStrings::_wstring_value_default2_type arg)
  {
    msg_.wstring_value_default2 = std::move(arg);
    return Init_WStrings_wstring_value_default3(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::WStrings msg_;
};

class Init_WStrings_wstring_value_default1
{
public:
  explicit Init_WStrings_wstring_value_default1(::rosidl_generator_cpp::msg::WStrings & msg)
  : msg_(msg)
  {}
  Init_WStrings_wstring_value_default2 wstring_value_default1(::rosidl_generator_cpp::msg::WStrings::_wstring_value_default1_type arg)
  {
    msg_.wstring_value_default1 = std::move(arg);
    return Init_WStrings_wstring_value_default2(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::WStrings msg_;
};

class Init_WStrings_wstring_value
{
public:
  Init_WStrings_wstring_value()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WStrings_wstring_value_default1 wstring_value(::rosidl_generator_cpp::msg::WStrings::_wstring_value_type arg)
  {
    msg_.wstring_value = std::move(arg);
    return Init_WStrings_wstring_value_default1(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::WStrings msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosidl_generator_cpp::msg::WStrings>()
{
  return rosidl_generator_cpp::msg::builder::Init_WStrings_wstring_value();
}

}  // namespace rosidl_generator_cpp

#endif  // ROSIDL_GENERATOR_CPP__MSG__DETAIL__W_STRINGS__BUILDER_HPP_
