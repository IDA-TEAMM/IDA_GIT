// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosidl_generator_cpp:msg/Empty.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_GENERATOR_CPP__MSG__DETAIL__EMPTY__BUILDER_HPP_
#define ROSIDL_GENERATOR_CPP__MSG__DETAIL__EMPTY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosidl_generator_cpp/msg/detail/empty__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosidl_generator_cpp
{

namespace msg
{


}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosidl_generator_cpp::msg::Empty>()
{
  return ::rosidl_generator_cpp::msg::Empty(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rosidl_generator_cpp

#endif  // ROSIDL_GENERATOR_CPP__MSG__DETAIL__EMPTY__BUILDER_HPP_
