// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosidl_generator_cpp:msg/BasicTypes.idl
// generated code does not contain a copyright notice

#ifndef ROSIDL_GENERATOR_CPP__MSG__DETAIL__BASIC_TYPES__BUILDER_HPP_
#define ROSIDL_GENERATOR_CPP__MSG__DETAIL__BASIC_TYPES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosidl_generator_cpp/msg/detail/basic_types__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosidl_generator_cpp
{

namespace msg
{

namespace builder
{

class Init_BasicTypes_uint64_value
{
public:
  explicit Init_BasicTypes_uint64_value(::rosidl_generator_cpp::msg::BasicTypes & msg)
  : msg_(msg)
  {}
  ::rosidl_generator_cpp::msg::BasicTypes uint64_value(::rosidl_generator_cpp::msg::BasicTypes::_uint64_value_type arg)
  {
    msg_.uint64_value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::BasicTypes msg_;
};

class Init_BasicTypes_int64_value
{
public:
  explicit Init_BasicTypes_int64_value(::rosidl_generator_cpp::msg::BasicTypes & msg)
  : msg_(msg)
  {}
  Init_BasicTypes_uint64_value int64_value(::rosidl_generator_cpp::msg::BasicTypes::_int64_value_type arg)
  {
    msg_.int64_value = std::move(arg);
    return Init_BasicTypes_uint64_value(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::BasicTypes msg_;
};

class Init_BasicTypes_uint32_value
{
public:
  explicit Init_BasicTypes_uint32_value(::rosidl_generator_cpp::msg::BasicTypes & msg)
  : msg_(msg)
  {}
  Init_BasicTypes_int64_value uint32_value(::rosidl_generator_cpp::msg::BasicTypes::_uint32_value_type arg)
  {
    msg_.uint32_value = std::move(arg);
    return Init_BasicTypes_int64_value(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::BasicTypes msg_;
};

class Init_BasicTypes_int32_value
{
public:
  explicit Init_BasicTypes_int32_value(::rosidl_generator_cpp::msg::BasicTypes & msg)
  : msg_(msg)
  {}
  Init_BasicTypes_uint32_value int32_value(::rosidl_generator_cpp::msg::BasicTypes::_int32_value_type arg)
  {
    msg_.int32_value = std::move(arg);
    return Init_BasicTypes_uint32_value(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::BasicTypes msg_;
};

class Init_BasicTypes_uint16_value
{
public:
  explicit Init_BasicTypes_uint16_value(::rosidl_generator_cpp::msg::BasicTypes & msg)
  : msg_(msg)
  {}
  Init_BasicTypes_int32_value uint16_value(::rosidl_generator_cpp::msg::BasicTypes::_uint16_value_type arg)
  {
    msg_.uint16_value = std::move(arg);
    return Init_BasicTypes_int32_value(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::BasicTypes msg_;
};

class Init_BasicTypes_int16_value
{
public:
  explicit Init_BasicTypes_int16_value(::rosidl_generator_cpp::msg::BasicTypes & msg)
  : msg_(msg)
  {}
  Init_BasicTypes_uint16_value int16_value(::rosidl_generator_cpp::msg::BasicTypes::_int16_value_type arg)
  {
    msg_.int16_value = std::move(arg);
    return Init_BasicTypes_uint16_value(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::BasicTypes msg_;
};

class Init_BasicTypes_uint8_value
{
public:
  explicit Init_BasicTypes_uint8_value(::rosidl_generator_cpp::msg::BasicTypes & msg)
  : msg_(msg)
  {}
  Init_BasicTypes_int16_value uint8_value(::rosidl_generator_cpp::msg::BasicTypes::_uint8_value_type arg)
  {
    msg_.uint8_value = std::move(arg);
    return Init_BasicTypes_int16_value(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::BasicTypes msg_;
};

class Init_BasicTypes_int8_value
{
public:
  explicit Init_BasicTypes_int8_value(::rosidl_generator_cpp::msg::BasicTypes & msg)
  : msg_(msg)
  {}
  Init_BasicTypes_uint8_value int8_value(::rosidl_generator_cpp::msg::BasicTypes::_int8_value_type arg)
  {
    msg_.int8_value = std::move(arg);
    return Init_BasicTypes_uint8_value(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::BasicTypes msg_;
};

class Init_BasicTypes_float64_value
{
public:
  explicit Init_BasicTypes_float64_value(::rosidl_generator_cpp::msg::BasicTypes & msg)
  : msg_(msg)
  {}
  Init_BasicTypes_int8_value float64_value(::rosidl_generator_cpp::msg::BasicTypes::_float64_value_type arg)
  {
    msg_.float64_value = std::move(arg);
    return Init_BasicTypes_int8_value(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::BasicTypes msg_;
};

class Init_BasicTypes_float32_value
{
public:
  explicit Init_BasicTypes_float32_value(::rosidl_generator_cpp::msg::BasicTypes & msg)
  : msg_(msg)
  {}
  Init_BasicTypes_float64_value float32_value(::rosidl_generator_cpp::msg::BasicTypes::_float32_value_type arg)
  {
    msg_.float32_value = std::move(arg);
    return Init_BasicTypes_float64_value(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::BasicTypes msg_;
};

class Init_BasicTypes_char_value
{
public:
  explicit Init_BasicTypes_char_value(::rosidl_generator_cpp::msg::BasicTypes & msg)
  : msg_(msg)
  {}
  Init_BasicTypes_float32_value char_value(::rosidl_generator_cpp::msg::BasicTypes::_char_value_type arg)
  {
    msg_.char_value = std::move(arg);
    return Init_BasicTypes_float32_value(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::BasicTypes msg_;
};

class Init_BasicTypes_byte_value
{
public:
  explicit Init_BasicTypes_byte_value(::rosidl_generator_cpp::msg::BasicTypes & msg)
  : msg_(msg)
  {}
  Init_BasicTypes_char_value byte_value(::rosidl_generator_cpp::msg::BasicTypes::_byte_value_type arg)
  {
    msg_.byte_value = std::move(arg);
    return Init_BasicTypes_char_value(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::BasicTypes msg_;
};

class Init_BasicTypes_bool_value
{
public:
  Init_BasicTypes_bool_value()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BasicTypes_byte_value bool_value(::rosidl_generator_cpp::msg::BasicTypes::_bool_value_type arg)
  {
    msg_.bool_value = std::move(arg);
    return Init_BasicTypes_byte_value(msg_);
  }

private:
  ::rosidl_generator_cpp::msg::BasicTypes msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosidl_generator_cpp::msg::BasicTypes>()
{
  return rosidl_generator_cpp::msg::builder::Init_BasicTypes_bool_value();
}

}  // namespace rosidl_generator_cpp

#endif  // ROSIDL_GENERATOR_CPP__MSG__DETAIL__BASIC_TYPES__BUILDER_HPP_
