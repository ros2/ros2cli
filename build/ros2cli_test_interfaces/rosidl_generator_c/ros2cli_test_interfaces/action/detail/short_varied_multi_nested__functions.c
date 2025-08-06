// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2cli_test_interfaces:action/ShortVariedMultiNested.idl
// generated code does not contain a copyright notice
#include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `short_varied_nested`
#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__functions.h"

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // short_varied_nested
  if (!ros2cli_test_interfaces__msg__ShortVariedNested__init(&msg->short_varied_nested)) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__fini(msg);
    return false;
  }
  return true;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * msg)
{
  if (!msg) {
    return;
  }
  // short_varied_nested
  ros2cli_test_interfaces__msg__ShortVariedNested__fini(&msg->short_varied_nested);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // short_varied_nested
  if (!ros2cli_test_interfaces__msg__ShortVariedNested__are_equal(
      &(lhs->short_varied_nested), &(rhs->short_varied_nested)))
  {
    return false;
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // short_varied_nested
  if (!ros2cli_test_interfaces__msg__ShortVariedNested__copy(
      &(input->short_varied_nested), &(output->short_varied_nested)))
  {
    return false;
  }
  return true;
}

ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal *
ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * msg = (ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal));
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * data = NULL;

  if (size) {
    data = (ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal *)allocator.zero_allocate(size, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__fini(&data[i - 1]);
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
ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence * array)
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
      ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__fini(&array->data[i]);
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

ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence *
ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence * array = (ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * data =
      (ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * msg)
{
  if (!msg) {
    return false;
  }
  // bool_value
  return true;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * msg)
{
  if (!msg) {
    return;
  }
  // bool_value
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // bool_value
  if (lhs->bool_value != rhs->bool_value) {
    return false;
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // bool_value
  output->bool_value = input->bool_value;
  return true;
}

ros2cli_test_interfaces__action__ShortVariedMultiNested_Result *
ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * msg = (ros2cli_test_interfaces__action__ShortVariedMultiNested_Result *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_Result));
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * data = NULL;

  if (size) {
    data = (ros2cli_test_interfaces__action__ShortVariedMultiNested_Result *)allocator.zero_allocate(size, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__fini(&data[i - 1]);
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
ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence * array)
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
      ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__fini(&array->data[i]);
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

ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence *
ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence * array = (ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * data =
      (ros2cli_test_interfaces__action__ShortVariedMultiNested_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // bool_values
  return true;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // bool_values
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // bool_values
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->bool_values[i] != rhs->bool_values[i]) {
      return false;
    }
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // bool_values
  for (size_t i = 0; i < 3; ++i) {
    output->bool_values[i] = input->bool_values[i];
  }
  return true;
}

ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback *
ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * msg = (ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback));
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * data = NULL;

  if (size) {
    data = (ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback *)allocator.zero_allocate(size, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__fini(&data[i - 1]);
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
ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence * array)
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
      ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__fini(&array->data[i]);
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

ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence *
ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence * array = (ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * data =
      (ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__init(&msg->goal)) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__fini(&msg->goal);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request *
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * msg = (ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request));
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * data = NULL;

  if (size) {
    data = (ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request *)allocator.zero_allocate(size, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__fini(&data[i - 1]);
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
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence * array)
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
      ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__fini(&array->data[i]);
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

ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence *
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence * array = (ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * data =
      (ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response *
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * msg = (ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response));
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * data = NULL;

  if (size) {
    data = (ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response *)allocator.zero_allocate(size, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__fini(&data[i - 1]);
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
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence * array)
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
      ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__fini(&array->data[i]);
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

ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence *
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence * array = (ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * data =
      (ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__fini(msg);
    return false;
  }
  // request
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence__init(&msg->request, 0)) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__fini(msg);
    return false;
  }
  // response
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence__init(&msg->response, 0)) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__fini(msg);
    return false;
  }
  return true;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence__fini(&msg->request);
  // response
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence__fini(&msg->response);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event *
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event * msg = (ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event));
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event * data = NULL;

  if (size) {
    data = (ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event *)allocator.zero_allocate(size, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__fini(&data[i - 1]);
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
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence * array)
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
      ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__fini(&array->data[i]);
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

ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence *
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence * array = (ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event * data =
      (ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request *
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * msg = (ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request));
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * data = NULL;

  if (size) {
    data = (ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request *)allocator.zero_allocate(size, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__fini(&data[i - 1]);
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
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence * array)
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
      ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__fini(&array->data[i]);
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

ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence *
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence * array = (ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * data =
      (ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__init(&msg->result)) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__fini(&msg->result);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response *
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * msg = (ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response));
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * data = NULL;

  if (size) {
    data = (ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response *)allocator.zero_allocate(size, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__fini(&data[i - 1]);
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
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence * array)
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
      ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__fini(&array->data[i]);
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

ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence *
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence * array = (ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * data =
      (ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
// already included above
// #include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__fini(msg);
    return false;
  }
  // request
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence__init(&msg->request, 0)) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__fini(msg);
    return false;
  }
  // response
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence__init(&msg->response, 0)) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__fini(msg);
    return false;
  }
  return true;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence__fini(&msg->request);
  // response
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence__fini(&msg->response);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event *
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event * msg = (ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event));
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event * data = NULL;

  if (size) {
    data = (ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event *)allocator.zero_allocate(size, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__fini(&data[i - 1]);
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
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence * array)
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
      ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__fini(&array->data[i]);
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

ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence *
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence * array = (ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event * data =
      (ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__init(&msg->feedback)) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__fini(&msg->feedback);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage *
ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage * msg = (ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage));
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence__init(ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage * data = NULL;

  if (size) {
    data = (ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage *)allocator.zero_allocate(size, sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__fini(&data[i - 1]);
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
ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence__fini(ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence * array)
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
      ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__fini(&array->data[i]);
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

ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence *
ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence * array = (ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence *)allocator.allocate(sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence__destroy(ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence__are_equal(const ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence * lhs, const ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence__copy(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence * input,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage * data =
      (ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
