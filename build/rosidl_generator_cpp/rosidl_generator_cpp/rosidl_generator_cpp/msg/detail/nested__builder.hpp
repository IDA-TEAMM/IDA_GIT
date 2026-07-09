// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosidl_generator_cpp:msg/Nested.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_GENERATOR_CPP__MSG__DETAIL__NESTED__BUILDER_HPP_
#define ROSIDL_GENERATOR_CPP__MSG__DETAIL__NESTED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosidl_generator_cpp/msg/detail/nested__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosidl_generator_cpp
{

namespace msg
{

namespace builder
{

class Init_Nested_basic_types_value
{
public:
  Init_Nested_basic_types_value()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosidl_generator_cpp::msg::Nested basic_types_value(::rosidl_generator_cpp::msg::Nested::_basic_types_value_type arg)
  {
    msg_.basic_types_value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::Nested msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosidl_generator_cpp::msg::Nested>()
{
  return rosidl_generator_cpp::msg::builder::Init_Nested_basic_types_value();
}

}  // namespace rosidl_generator_cpp

#endif  // ROSIDL_GENERATOR_CPP__MSG__DETAIL__NESTED__BUILDER_HPP_
