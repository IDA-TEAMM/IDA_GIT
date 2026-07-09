// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosidl_generator_cpp:srv/Empty.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_GENERATOR_CPP__SRV__DETAIL__EMPTY__BUILDER_HPP_
#define ROSIDL_GENERATOR_CPP__SRV__DETAIL__EMPTY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosidl_generator_cpp/srv/detail/empty__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosidl_generator_cpp
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosidl_generator_cpp::srv::Empty_Request>()
{
  return ::rosidl_generator_cpp::srv::Empty_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rosidl_generator_cpp


namespace rosidl_generator_cpp
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosidl_generator_cpp::srv::Empty_Response>()
{
  return ::rosidl_generator_cpp::srv::Empty_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rosidl_generator_cpp

#endif  // ROSIDL_GENERATOR_CPP__SRV__DETAIL__EMPTY__BUILDER_HPP_
