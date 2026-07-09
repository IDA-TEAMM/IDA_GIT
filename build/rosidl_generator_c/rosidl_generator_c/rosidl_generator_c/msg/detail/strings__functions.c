// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosidl_generator_c:msg/Strings.idl
// generated code does not contain a copyright notice
#include "rosidl_generator_c/msg/detail/strings__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `string_value`
// Member `string_value_default1`
// Member `string_value_default2`
// Member `string_value_default3`
// Member `string_value_default4`
// Member `string_value_default5`
// Member `bounded_string_value`
// Member `bounded_string_value_default1`
// Member `bounded_string_value_default2`
// Member `bounded_string_value_default3`
// Member `bounded_string_value_default4`
// Member `bounded_string_value_default5`
#include "rosidl_runtime_c/string_functions.h"

bool
rosidl_generator_c__msg__Strings__init(rosidl_generator_c__msg__Strings * msg)
{
  if (!msg) {
    return false;
  }
  // string_value
  if (!rosidl_runtime_c__String__init(&msg->string_value)) {
    rosidl_generator_c__msg__Strings__fini(msg);
    return false;
  }
  // string_value_default1
  if (!rosidl_runtime_c__String__init(&msg->string_value_default1)) {
    rosidl_generator_c__msg__Strings__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->string_value_default1, "Hello world!");
    if (!success) {
      goto abort_init_0;
    }
  }
  // string_value_default2
  if (!rosidl_runtime_c__String__init(&msg->string_value_default2)) {
    rosidl_generator_c__msg__Strings__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->string_value_default2, "Hello'world!");
    if (!success) {
      goto abort_init_1;
    }
  }
  // string_value_default3
  if (!rosidl_runtime_c__String__init(&msg->string_value_default3)) {
    rosidl_generator_c__msg__Strings__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->string_value_default3, "Hello\"world!");
    if (!success) {
      goto abort_init_2;
    }
  }
  // string_value_default4
  if (!rosidl_runtime_c__String__init(&msg->string_value_default4)) {
    rosidl_generator_c__msg__Strings__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->string_value_default4, "Hello'world!");
    if (!success) {
      goto abort_init_3;
    }
  }
  // string_value_default5
  if (!rosidl_runtime_c__String__init(&msg->string_value_default5)) {
    rosidl_generator_c__msg__Strings__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->string_value_default5, "Hello\"world!");
    if (!success) {
      goto abort_init_4;
    }
  }
  // bounded_string_value
  if (!rosidl_runtime_c__String__init(&msg->bounded_string_value)) {
    rosidl_generator_c__msg__Strings__fini(msg);
    return false;
  }
  // bounded_string_value_default1
  if (!rosidl_runtime_c__String__init(&msg->bounded_string_value_default1)) {
    rosidl_generator_c__msg__Strings__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->bounded_string_value_default1, "Hello world!");
    if (!success) {
      goto abort_init_5;
    }
  }
  // bounded_string_value_default2
  if (!rosidl_runtime_c__String__init(&msg->bounded_string_value_default2)) {
    rosidl_generator_c__msg__Strings__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->bounded_string_value_default2, "Hello'world!");
    if (!success) {
      goto abort_init_6;
    }
  }
  // bounded_string_value_default3
  if (!rosidl_runtime_c__String__init(&msg->bounded_string_value_default3)) {
    rosidl_generator_c__msg__Strings__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->bounded_string_value_default3, "Hello\"world!");
    if (!success) {
      goto abort_init_7;
    }
  }
  // bounded_string_value_default4
  if (!rosidl_runtime_c__String__init(&msg->bounded_string_value_default4)) {
    rosidl_generator_c__msg__Strings__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->bounded_string_value_default4, "Hello'world!");
    if (!success) {
      goto abort_init_8;
    }
  }
  // bounded_string_value_default5
  if (!rosidl_runtime_c__String__init(&msg->bounded_string_value_default5)) {
    rosidl_generator_c__msg__Strings__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->bounded_string_value_default5, "Hello\"world!");
    if (!success) {
      goto abort_init_9;
    }
  }
  return true;
abort_init_9:
  rosidl_runtime_c__String__fini(&msg->bounded_string_value_default4);
abort_init_8:
  rosidl_runtime_c__String__fini(&msg->bounded_string_value_default3);
abort_init_7:
  rosidl_runtime_c__String__fini(&msg->bounded_string_value_default2);
abort_init_6:
  rosidl_runtime_c__String__fini(&msg->bounded_string_value_default1);
abort_init_5:
  rosidl_runtime_c__String__fini(&msg->string_value_default5);
abort_init_4:
  rosidl_runtime_c__String__fini(&msg->string_value_default4);
abort_init_3:
  rosidl_runtime_c__String__fini(&msg->string_value_default3);
abort_init_2:
  rosidl_runtime_c__String__fini(&msg->string_value_default2);
abort_init_1:
  rosidl_runtime_c__String__fini(&msg->string_value_default1);
abort_init_0:
  return false;
}

void
rosidl_generator_c__msg__Strings__fini(rosidl_generator_c__msg__Strings * msg)
{
  if (!msg) {
    return;
  }
  // string_value
  rosidl_runtime_c__String__fini(&msg->string_value);
  // string_value_default1
  rosidl_runtime_c__String__fini(&msg->string_value_default1);
  // string_value_default2
  rosidl_runtime_c__String__fini(&msg->string_value_default2);
  // string_value_default3
  rosidl_runtime_c__String__fini(&msg->string_value_default3);
  // string_value_default4
  rosidl_runtime_c__String__fini(&msg->string_value_default4);
  // string_value_default5
  rosidl_runtime_c__String__fini(&msg->string_value_default5);
  // bounded_string_value
  rosidl_runtime_c__String__fini(&msg->bounded_string_value);
  // bounded_string_value_default1
  rosidl_runtime_c__String__fini(&msg->bounded_string_value_default1);
  // bounded_string_value_default2
  rosidl_runtime_c__String__fini(&msg->bounded_string_value_default2);
  // bounded_string_value_default3
  rosidl_runtime_c__String__fini(&msg->bounded_string_value_default3);
  // bounded_string_value_default4
  rosidl_runtime_c__String__fini(&msg->bounded_string_value_default4);
  // bounded_string_value_default5
  rosidl_runtime_c__String__fini(&msg->bounded_string_value_default5);
}

bool
rosidl_generator_c__msg__Strings__are_equal(const rosidl_generator_c__msg__Strings * lhs, const rosidl_generator_c__msg__Strings * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // string_value
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->string_value), &(rhs->string_value)))
  {
    return false;
  }
  // string_value_default1
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->string_value_default1), &(rhs->string_value_default1)))
  {
    return false;
  }
  // string_value_default2
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->string_value_default2), &(rhs->string_value_default2)))
  {
    return false;
  }
  // string_value_default3
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->string_value_default3), &(rhs->string_value_default3)))
  {
    return false;
  }
  // string_value_default4
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->string_value_default4), &(rhs->string_value_default4)))
  {
    return false;
  }
  // string_value_default5
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->string_value_default5), &(rhs->string_value_default5)))
  {
    return false;
  }
  // bounded_string_value
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->bounded_string_value), &(rhs->bounded_string_value)))
  {
    return false;
  }
  // bounded_string_value_default1
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->bounded_string_value_default1), &(rhs->bounded_string_value_default1)))
  {
    return false;
  }
  // bounded_string_value_default2
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->bounded_string_value_default2), &(rhs->bounded_string_value_default2)))
  {
    return false;
  }
  // bounded_string_value_default3
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->bounded_string_value_default3), &(rhs->bounded_string_value_default3)))
  {
    return false;
  }
  // bounded_string_value_default4
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->bounded_string_value_default4), &(rhs->bounded_string_value_default4)))
  {
    return false;
  }
  // bounded_string_value_default5
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->bounded_string_value_default5), &(rhs->bounded_string_value_default5)))
  {
    return false;
  }
  return true;
}

bool
rosidl_generator_c__msg__Strings__copy(
  const rosidl_generator_c__msg__Strings * input,
  rosidl_generator_c__msg__Strings * output)
{
  if (!input || !output) {
    return false;
  }
  // string_value
  if (!rosidl_runtime_c__String__copy(
      &(input->string_value), &(output->string_value)))
  {
    return false;
  }
  // string_value_default1
  if (!rosidl_runtime_c__String__copy(
      &(input->string_value_default1), &(output->string_value_default1)))
  {
    return false;
  }
  // string_value_default2
  if (!rosidl_runtime_c__String__copy(
      &(input->string_value_default2), &(output->string_value_default2)))
  {
    return false;
  }
  // string_value_default3
  if (!rosidl_runtime_c__String__copy(
      &(input->string_value_default3), &(output->string_value_default3)))
  {
    return false;
  }
  // string_value_default4
  if (!rosidl_runtime_c__String__copy(
      &(input->string_value_default4), &(output->string_value_default4)))
  {
    return false;
  }
  // string_value_default5
  if (!rosidl_runtime_c__String__copy(
      &(input->string_value_default5), &(output->string_value_default5)))
  {
    return false;
  }
  // bounded_string_value
  if (!rosidl_runtime_c__String__copy(
      &(input->bounded_string_value), &(output->bounded_string_value)))
  {
    return false;
  }
  // bounded_string_value_default1
  if (!rosidl_runtime_c__String__copy(
      &(input->bounded_string_value_default1), &(output->bounded_string_value_default1)))
  {
    return false;
  }
  // bounded_string_value_default2
  if (!rosidl_runtime_c__String__copy(
      &(input->bounded_string_value_default2), &(output->bounded_string_value_default2)))
  {
    return false;
  }
  // bounded_string_value_default3
  if (!rosidl_runtime_c__String__copy(
      &(input->bounded_string_value_default3), &(output->bounded_string_value_default3)))
  {
    return false;
  }
  // bounded_string_value_default4
  if (!rosidl_runtime_c__String__copy(
      &(input->bounded_string_value_default4), &(output->bounded_string_value_default4)))
  {
    return false;
  }
  // bounded_string_value_default5
  if (!rosidl_runtime_c__String__copy(
      &(input->bounded_string_value_default5), &(output->bounded_string_value_default5)))
  {
    return false;
  }
  return true;
}

rosidl_generator_c__msg__Strings *
rosidl_generator_c__msg__Strings__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_generator_c__msg__Strings * msg = (rosidl_generator_c__msg__Strings *)allocator.allocate(sizeof(rosidl_generator_c__msg__Strings), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosidl_generator_c__msg__Strings));
  bool success = rosidl_generator_c__msg__Strings__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosidl_generator_c__msg__Strings__destroy(rosidl_generator_c__msg__Strings * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosidl_generator_c__msg__Strings__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosidl_generator_c__msg__Strings__Sequence__init(rosidl_generator_c__msg__Strings__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_generator_c__msg__Strings * data = NULL;

  if (size) {
    data = (rosidl_generator_c__msg__Strings *)allocator.zero_allocate(size, sizeof(rosidl_generator_c__msg__Strings), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosidl_generator_c__msg__Strings__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosidl_generator_c__msg__Strings__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rosidl_generator_c__msg__Strings__Sequence__fini(rosidl_generator_c__msg__Strings__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rosidl_generator_c__msg__Strings__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rosidl_generator_c__msg__Strings__Sequence *
rosidl_generator_c__msg__Strings__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_generator_c__msg__Strings__Sequence * array = (rosidl_generator_c__msg__Strings__Sequence *)allocator.allocate(sizeof(rosidl_generator_c__msg__Strings__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosidl_generator_c__msg__Strings__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosidl_generator_c__msg__Strings__Sequence__destroy(rosidl_generator_c__msg__Strings__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosidl_generator_c__msg__Strings__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosidl_generator_c__msg__Strings__Sequence__are_equal(const rosidl_generator_c__msg__Strings__Sequence * lhs, const rosidl_generator_c__msg__Strings__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosidl_generator_c__msg__Strings__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosidl_generator_c__msg__Strings__Sequence__copy(
  const rosidl_generator_c__msg__Strings__Sequence * input,
  rosidl_generator_c__msg__Strings__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosidl_generator_c__msg__Strings);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosidl_generator_c__msg__Strings * data =
      (rosidl_generator_c__msg__Strings *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosidl_generator_c__msg__Strings__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosidl_generator_c__msg__Strings__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosidl_generator_c__msg__Strings__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
