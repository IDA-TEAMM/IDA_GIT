// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosidl_generator_c:msg/MultiNested.idl
// generated code does not contain a copyright notice
#include "rosidl_generator_c/msg/detail/multi_nested__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `array_of_arrays`
// Member `bounded_sequence_of_arrays`
// Member `unbounded_sequence_of_arrays`
#include "rosidl_generator_c/msg/detail/arrays__functions.h"
// Member `array_of_bounded_sequences`
// Member `bounded_sequence_of_bounded_sequences`
// Member `unbounded_sequence_of_bounded_sequences`
#include "rosidl_generator_c/msg/detail/bounded_sequences__functions.h"
// Member `array_of_unbounded_sequences`
// Member `bounded_sequence_of_unbounded_sequences`
// Member `unbounded_sequence_of_unbounded_sequences`
#include "rosidl_generator_c/msg/detail/unbounded_sequences__functions.h"

bool
rosidl_generator_c__msg__MultiNested__init(rosidl_generator_c__msg__MultiNested * msg)
{
  if (!msg) {
    return false;
  }
  // array_of_arrays
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_generator_c__msg__Arrays__init(&msg->array_of_arrays[i])) {
      rosidl_generator_c__msg__MultiNested__fini(msg);
      return false;
    }
  }
  // array_of_bounded_sequences
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_generator_c__msg__BoundedSequences__init(&msg->array_of_bounded_sequences[i])) {
      rosidl_generator_c__msg__MultiNested__fini(msg);
      return false;
    }
  }
  // array_of_unbounded_sequences
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_generator_c__msg__UnboundedSequences__init(&msg->array_of_unbounded_sequences[i])) {
      rosidl_generator_c__msg__MultiNested__fini(msg);
      return false;
    }
  }
  // bounded_sequence_of_arrays
  if (!rosidl_generator_c__msg__Arrays__Sequence__init(&msg->bounded_sequence_of_arrays, 0)) {
    rosidl_generator_c__msg__MultiNested__fini(msg);
    return false;
  }
  // bounded_sequence_of_bounded_sequences
  if (!rosidl_generator_c__msg__BoundedSequences__Sequence__init(&msg->bounded_sequence_of_bounded_sequences, 0)) {
    rosidl_generator_c__msg__MultiNested__fini(msg);
    return false;
  }
  // bounded_sequence_of_unbounded_sequences
  if (!rosidl_generator_c__msg__UnboundedSequences__Sequence__init(&msg->bounded_sequence_of_unbounded_sequences, 0)) {
    rosidl_generator_c__msg__MultiNested__fini(msg);
    return false;
  }
  // unbounded_sequence_of_arrays
  if (!rosidl_generator_c__msg__Arrays__Sequence__init(&msg->unbounded_sequence_of_arrays, 0)) {
    rosidl_generator_c__msg__MultiNested__fini(msg);
    return false;
  }
  // unbounded_sequence_of_bounded_sequences
  if (!rosidl_generator_c__msg__BoundedSequences__Sequence__init(&msg->unbounded_sequence_of_bounded_sequences, 0)) {
    rosidl_generator_c__msg__MultiNested__fini(msg);
    return false;
  }
  // unbounded_sequence_of_unbounded_sequences
  if (!rosidl_generator_c__msg__UnboundedSequences__Sequence__init(&msg->unbounded_sequence_of_unbounded_sequences, 0)) {
    rosidl_generator_c__msg__MultiNested__fini(msg);
    return false;
  }
  return true;
}

void
rosidl_generator_c__msg__MultiNested__fini(rosidl_generator_c__msg__MultiNested * msg)
{
  if (!msg) {
    return;
  }
  // array_of_arrays
  for (size_t i = 0; i < 3; ++i) {
    rosidl_generator_c__msg__Arrays__fini(&msg->array_of_arrays[i]);
  }
  // array_of_bounded_sequences
  for (size_t i = 0; i < 3; ++i) {
    rosidl_generator_c__msg__BoundedSequences__fini(&msg->array_of_bounded_sequences[i]);
  }
  // array_of_unbounded_sequences
  for (size_t i = 0; i < 3; ++i) {
    rosidl_generator_c__msg__UnboundedSequences__fini(&msg->array_of_unbounded_sequences[i]);
  }
  // bounded_sequence_of_arrays
  rosidl_generator_c__msg__Arrays__Sequence__fini(&msg->bounded_sequence_of_arrays);
  // bounded_sequence_of_bounded_sequences
  rosidl_generator_c__msg__BoundedSequences__Sequence__fini(&msg->bounded_sequence_of_bounded_sequences);
  // bounded_sequence_of_unbounded_sequences
  rosidl_generator_c__msg__UnboundedSequences__Sequence__fini(&msg->bounded_sequence_of_unbounded_sequences);
  // unbounded_sequence_of_arrays
  rosidl_generator_c__msg__Arrays__Sequence__fini(&msg->unbounded_sequence_of_arrays);
  // unbounded_sequence_of_bounded_sequences
  rosidl_generator_c__msg__BoundedSequences__Sequence__fini(&msg->unbounded_sequence_of_bounded_sequences);
  // unbounded_sequence_of_unbounded_sequences
  rosidl_generator_c__msg__UnboundedSequences__Sequence__fini(&msg->unbounded_sequence_of_unbounded_sequences);
}

bool
rosidl_generator_c__msg__MultiNested__are_equal(const rosidl_generator_c__msg__MultiNested * lhs, const rosidl_generator_c__msg__MultiNested * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // array_of_arrays
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_generator_c__msg__Arrays__are_equal(
        &(lhs->array_of_arrays[i]), &(rhs->array_of_arrays[i])))
    {
      return false;
    }
  }
  // array_of_bounded_sequences
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_generator_c__msg__BoundedSequences__are_equal(
        &(lhs->array_of_bounded_sequences[i]), &(rhs->array_of_bounded_sequences[i])))
    {
      return false;
    }
  }
  // array_of_unbounded_sequences
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_generator_c__msg__UnboundedSequences__are_equal(
        &(lhs->array_of_unbounded_sequences[i]), &(rhs->array_of_unbounded_sequences[i])))
    {
      return false;
    }
  }
  // bounded_sequence_of_arrays
  if (!rosidl_generator_c__msg__Arrays__Sequence__are_equal(
      &(lhs->bounded_sequence_of_arrays), &(rhs->bounded_sequence_of_arrays)))
  {
    return false;
  }
  // bounded_sequence_of_bounded_sequences
  if (!rosidl_generator_c__msg__BoundedSequences__Sequence__are_equal(
      &(lhs->bounded_sequence_of_bounded_sequences), &(rhs->bounded_sequence_of_bounded_sequences)))
  {
    return false;
  }
  // bounded_sequence_of_unbounded_sequences
  if (!rosidl_generator_c__msg__UnboundedSequences__Sequence__are_equal(
      &(lhs->bounded_sequence_of_unbounded_sequences), &(rhs->bounded_sequence_of_unbounded_sequences)))
  {
    return false;
  }
  // unbounded_sequence_of_arrays
  if (!rosidl_generator_c__msg__Arrays__Sequence__are_equal(
      &(lhs->unbounded_sequence_of_arrays), &(rhs->unbounded_sequence_of_arrays)))
  {
    return false;
  }
  // unbounded_sequence_of_bounded_sequences
  if (!rosidl_generator_c__msg__BoundedSequences__Sequence__are_equal(
      &(lhs->unbounded_sequence_of_bounded_sequences), &(rhs->unbounded_sequence_of_bounded_sequences)))
  {
    return false;
  }
  // unbounded_sequence_of_unbounded_sequences
  if (!rosidl_generator_c__msg__UnboundedSequences__Sequence__are_equal(
      &(lhs->unbounded_sequence_of_unbounded_sequences), &(rhs->unbounded_sequence_of_unbounded_sequences)))
  {
    return false;
  }
  return true;
}

bool
rosidl_generator_c__msg__MultiNested__copy(
  const rosidl_generator_c__msg__MultiNested * input,
  rosidl_generator_c__msg__MultiNested * output)
{
  if (!input || !output) {
    return false;
  }
  // array_of_arrays
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_generator_c__msg__Arrays__copy(
        &(input->array_of_arrays[i]), &(output->array_of_arrays[i])))
    {
      return false;
    }
  }
  // array_of_bounded_sequences
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_generator_c__msg__BoundedSequences__copy(
        &(input->array_of_bounded_sequences[i]), &(output->array_of_bounded_sequences[i])))
    {
      return false;
    }
  }
  // array_of_unbounded_sequences
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_generator_c__msg__UnboundedSequences__copy(
        &(input->array_of_unbounded_sequences[i]), &(output->array_of_unbounded_sequences[i])))
    {
      return false;
    }
  }
  // bounded_sequence_of_arrays
  if (!rosidl_generator_c__msg__Arrays__Sequence__copy(
      &(input->bounded_sequence_of_arrays), &(output->bounded_sequence_of_arrays)))
  {
    return false;
  }
  // bounded_sequence_of_bounded_sequences
  if (!rosidl_generator_c__msg__BoundedSequences__Sequence__copy(
      &(input->bounded_sequence_of_bounded_sequences), &(output->bounded_sequence_of_bounded_sequences)))
  {
    return false;
  }
  // bounded_sequence_of_unbounded_sequences
  if (!rosidl_generator_c__msg__UnboundedSequences__Sequence__copy(
      &(input->bounded_sequence_of_unbounded_sequences), &(output->bounded_sequence_of_unbounded_sequences)))
  {
    return false;
  }
  // unbounded_sequence_of_arrays
  if (!rosidl_generator_c__msg__Arrays__Sequence__copy(
      &(input->unbounded_sequence_of_arrays), &(output->unbounded_sequence_of_arrays)))
  {
    return false;
  }
  // unbounded_sequence_of_bounded_sequences
  if (!rosidl_generator_c__msg__BoundedSequences__Sequence__copy(
      &(input->unbounded_sequence_of_bounded_sequences), &(output->unbounded_sequence_of_bounded_sequences)))
  {
    return false;
  }
  // unbounded_sequence_of_unbounded_sequences
  if (!rosidl_generator_c__msg__UnboundedSequences__Sequence__copy(
      &(input->unbounded_sequence_of_unbounded_sequences), &(output->unbounded_sequence_of_unbounded_sequences)))
  {
    return false;
  }
  return true;
}

rosidl_generator_c__msg__MultiNested *
rosidl_generator_c__msg__MultiNested__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_generator_c__msg__MultiNested * msg = (rosidl_generator_c__msg__MultiNested *)allocator.allocate(sizeof(rosidl_generator_c__msg__MultiNested), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosidl_generator_c__msg__MultiNested));
  bool success = rosidl_generator_c__msg__MultiNested__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosidl_generator_c__msg__MultiNested__destroy(rosidl_generator_c__msg__MultiNested * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosidl_generator_c__msg__MultiNested__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosidl_generator_c__msg__MultiNested__Sequence__init(rosidl_generator_c__msg__MultiNested__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_generator_c__msg__MultiNested * data = NULL;

  if (size) {
    data = (rosidl_generator_c__msg__MultiNested *)allocator.zero_allocate(size, sizeof(rosidl_generator_c__msg__MultiNested), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosidl_generator_c__msg__MultiNested__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosidl_generator_c__msg__MultiNested__fini(&data[i - 1]);
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
rosidl_generator_c__msg__MultiNested__Sequence__fini(rosidl_generator_c__msg__MultiNested__Sequence * array)
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
      rosidl_generator_c__msg__MultiNested__fini(&array->data[i]);
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

rosidl_generator_c__msg__MultiNested__Sequence *
rosidl_generator_c__msg__MultiNested__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_generator_c__msg__MultiNested__Sequence * array = (rosidl_generator_c__msg__MultiNested__Sequence *)allocator.allocate(sizeof(rosidl_generator_c__msg__MultiNested__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosidl_generator_c__msg__MultiNested__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosidl_generator_c__msg__MultiNested__Sequence__destroy(rosidl_generator_c__msg__MultiNested__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosidl_generator_c__msg__MultiNested__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosidl_generator_c__msg__MultiNested__Sequence__are_equal(const rosidl_generator_c__msg__MultiNested__Sequence * lhs, const rosidl_generator_c__msg__MultiNested__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosidl_generator_c__msg__MultiNested__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosidl_generator_c__msg__MultiNested__Sequence__copy(
  const rosidl_generator_c__msg__MultiNested__Sequence * input,
  rosidl_generator_c__msg__MultiNested__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosidl_generator_c__msg__MultiNested);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosidl_generator_c__msg__MultiNested * data =
      (rosidl_generator_c__msg__MultiNested *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosidl_generator_c__msg__MultiNested__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosidl_generator_c__msg__MultiNested__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosidl_generator_c__msg__MultiNested__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
