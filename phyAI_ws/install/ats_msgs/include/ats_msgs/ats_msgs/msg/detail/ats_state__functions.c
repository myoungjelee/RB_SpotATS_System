// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ats_msgs:msg/AtsState.idl
// generated code does not contain a copyright notice
#include "ats_msgs/msg/detail/ats_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `system_state`
// Member `pose_frame`
// Member `vision_json`
// Member `plan_json`
// Member `queue_status`
// Member `history_json`
// Member `last_violation`
// Member `events_json`
#include "rosidl_runtime_c/string_functions.h"

bool
ats_msgs__msg__AtsState__init(ats_msgs__msg__AtsState * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    ats_msgs__msg__AtsState__fini(msg);
    return false;
  }
  // system_state
  if (!rosidl_runtime_c__String__init(&msg->system_state)) {
    ats_msgs__msg__AtsState__fini(msg);
    return false;
  }
  // pose_frame
  if (!rosidl_runtime_c__String__init(&msg->pose_frame)) {
    ats_msgs__msg__AtsState__fini(msg);
    return false;
  }
  // pose_x
  // pose_y
  // pose_yaw
  // vel_vx
  // vel_vy
  // vel_wz
  // battery_soc
  // battery_voltage
  // primary_id
  // lost_sec
  // vision_json
  if (!rosidl_runtime_c__String__init(&msg->vision_json)) {
    ats_msgs__msg__AtsState__fini(msg);
    return false;
  }
  // plan_json
  if (!rosidl_runtime_c__String__init(&msg->plan_json)) {
    ats_msgs__msg__AtsState__fini(msg);
    return false;
  }
  // current_index
  // queue_status
  if (!rosidl_runtime_c__String__init(&msg->queue_status)) {
    ats_msgs__msg__AtsState__fini(msg);
    return false;
  }
  // history_json
  if (!rosidl_runtime_c__String__init(&msg->history_json)) {
    ats_msgs__msg__AtsState__fini(msg);
    return false;
  }
  // roe_ok
  // safe_backstop
  // max_speed
  // last_violation
  if (!rosidl_runtime_c__String__init(&msg->last_violation)) {
    ats_msgs__msg__AtsState__fini(msg);
    return false;
  }
  // events_json
  if (!rosidl_runtime_c__String__init(&msg->events_json)) {
    ats_msgs__msg__AtsState__fini(msg);
    return false;
  }
  return true;
}

void
ats_msgs__msg__AtsState__fini(ats_msgs__msg__AtsState * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // system_state
  rosidl_runtime_c__String__fini(&msg->system_state);
  // pose_frame
  rosidl_runtime_c__String__fini(&msg->pose_frame);
  // pose_x
  // pose_y
  // pose_yaw
  // vel_vx
  // vel_vy
  // vel_wz
  // battery_soc
  // battery_voltage
  // primary_id
  // lost_sec
  // vision_json
  rosidl_runtime_c__String__fini(&msg->vision_json);
  // plan_json
  rosidl_runtime_c__String__fini(&msg->plan_json);
  // current_index
  // queue_status
  rosidl_runtime_c__String__fini(&msg->queue_status);
  // history_json
  rosidl_runtime_c__String__fini(&msg->history_json);
  // roe_ok
  // safe_backstop
  // max_speed
  // last_violation
  rosidl_runtime_c__String__fini(&msg->last_violation);
  // events_json
  rosidl_runtime_c__String__fini(&msg->events_json);
}

bool
ats_msgs__msg__AtsState__are_equal(const ats_msgs__msg__AtsState * lhs, const ats_msgs__msg__AtsState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  // system_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->system_state), &(rhs->system_state)))
  {
    return false;
  }
  // pose_frame
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->pose_frame), &(rhs->pose_frame)))
  {
    return false;
  }
  // pose_x
  if (lhs->pose_x != rhs->pose_x) {
    return false;
  }
  // pose_y
  if (lhs->pose_y != rhs->pose_y) {
    return false;
  }
  // pose_yaw
  if (lhs->pose_yaw != rhs->pose_yaw) {
    return false;
  }
  // vel_vx
  if (lhs->vel_vx != rhs->vel_vx) {
    return false;
  }
  // vel_vy
  if (lhs->vel_vy != rhs->vel_vy) {
    return false;
  }
  // vel_wz
  if (lhs->vel_wz != rhs->vel_wz) {
    return false;
  }
  // battery_soc
  if (lhs->battery_soc != rhs->battery_soc) {
    return false;
  }
  // battery_voltage
  if (lhs->battery_voltage != rhs->battery_voltage) {
    return false;
  }
  // primary_id
  if (lhs->primary_id != rhs->primary_id) {
    return false;
  }
  // lost_sec
  if (lhs->lost_sec != rhs->lost_sec) {
    return false;
  }
  // vision_json
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->vision_json), &(rhs->vision_json)))
  {
    return false;
  }
  // plan_json
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->plan_json), &(rhs->plan_json)))
  {
    return false;
  }
  // current_index
  if (lhs->current_index != rhs->current_index) {
    return false;
  }
  // queue_status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->queue_status), &(rhs->queue_status)))
  {
    return false;
  }
  // history_json
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->history_json), &(rhs->history_json)))
  {
    return false;
  }
  // roe_ok
  if (lhs->roe_ok != rhs->roe_ok) {
    return false;
  }
  // safe_backstop
  if (lhs->safe_backstop != rhs->safe_backstop) {
    return false;
  }
  // max_speed
  if (lhs->max_speed != rhs->max_speed) {
    return false;
  }
  // last_violation
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->last_violation), &(rhs->last_violation)))
  {
    return false;
  }
  // events_json
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->events_json), &(rhs->events_json)))
  {
    return false;
  }
  return true;
}

bool
ats_msgs__msg__AtsState__copy(
  const ats_msgs__msg__AtsState * input,
  ats_msgs__msg__AtsState * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  // system_state
  if (!rosidl_runtime_c__String__copy(
      &(input->system_state), &(output->system_state)))
  {
    return false;
  }
  // pose_frame
  if (!rosidl_runtime_c__String__copy(
      &(input->pose_frame), &(output->pose_frame)))
  {
    return false;
  }
  // pose_x
  output->pose_x = input->pose_x;
  // pose_y
  output->pose_y = input->pose_y;
  // pose_yaw
  output->pose_yaw = input->pose_yaw;
  // vel_vx
  output->vel_vx = input->vel_vx;
  // vel_vy
  output->vel_vy = input->vel_vy;
  // vel_wz
  output->vel_wz = input->vel_wz;
  // battery_soc
  output->battery_soc = input->battery_soc;
  // battery_voltage
  output->battery_voltage = input->battery_voltage;
  // primary_id
  output->primary_id = input->primary_id;
  // lost_sec
  output->lost_sec = input->lost_sec;
  // vision_json
  if (!rosidl_runtime_c__String__copy(
      &(input->vision_json), &(output->vision_json)))
  {
    return false;
  }
  // plan_json
  if (!rosidl_runtime_c__String__copy(
      &(input->plan_json), &(output->plan_json)))
  {
    return false;
  }
  // current_index
  output->current_index = input->current_index;
  // queue_status
  if (!rosidl_runtime_c__String__copy(
      &(input->queue_status), &(output->queue_status)))
  {
    return false;
  }
  // history_json
  if (!rosidl_runtime_c__String__copy(
      &(input->history_json), &(output->history_json)))
  {
    return false;
  }
  // roe_ok
  output->roe_ok = input->roe_ok;
  // safe_backstop
  output->safe_backstop = input->safe_backstop;
  // max_speed
  output->max_speed = input->max_speed;
  // last_violation
  if (!rosidl_runtime_c__String__copy(
      &(input->last_violation), &(output->last_violation)))
  {
    return false;
  }
  // events_json
  if (!rosidl_runtime_c__String__copy(
      &(input->events_json), &(output->events_json)))
  {
    return false;
  }
  return true;
}

ats_msgs__msg__AtsState *
ats_msgs__msg__AtsState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ats_msgs__msg__AtsState * msg = (ats_msgs__msg__AtsState *)allocator.allocate(sizeof(ats_msgs__msg__AtsState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ats_msgs__msg__AtsState));
  bool success = ats_msgs__msg__AtsState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ats_msgs__msg__AtsState__destroy(ats_msgs__msg__AtsState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ats_msgs__msg__AtsState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ats_msgs__msg__AtsState__Sequence__init(ats_msgs__msg__AtsState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ats_msgs__msg__AtsState * data = NULL;

  if (size) {
    data = (ats_msgs__msg__AtsState *)allocator.zero_allocate(size, sizeof(ats_msgs__msg__AtsState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ats_msgs__msg__AtsState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ats_msgs__msg__AtsState__fini(&data[i - 1]);
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
ats_msgs__msg__AtsState__Sequence__fini(ats_msgs__msg__AtsState__Sequence * array)
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
      ats_msgs__msg__AtsState__fini(&array->data[i]);
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

ats_msgs__msg__AtsState__Sequence *
ats_msgs__msg__AtsState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ats_msgs__msg__AtsState__Sequence * array = (ats_msgs__msg__AtsState__Sequence *)allocator.allocate(sizeof(ats_msgs__msg__AtsState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ats_msgs__msg__AtsState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ats_msgs__msg__AtsState__Sequence__destroy(ats_msgs__msg__AtsState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ats_msgs__msg__AtsState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ats_msgs__msg__AtsState__Sequence__are_equal(const ats_msgs__msg__AtsState__Sequence * lhs, const ats_msgs__msg__AtsState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ats_msgs__msg__AtsState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ats_msgs__msg__AtsState__Sequence__copy(
  const ats_msgs__msg__AtsState__Sequence * input,
  ats_msgs__msg__AtsState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ats_msgs__msg__AtsState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ats_msgs__msg__AtsState * data =
      (ats_msgs__msg__AtsState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ats_msgs__msg__AtsState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ats_msgs__msg__AtsState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ats_msgs__msg__AtsState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
