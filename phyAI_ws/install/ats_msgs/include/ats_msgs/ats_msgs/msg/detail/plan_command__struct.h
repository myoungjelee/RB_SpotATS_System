// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ats_msgs:msg/PlanCommand.idl
// generated code does not contain a copyright notice

#ifndef ATS_MSGS__MSG__DETAIL__PLAN_COMMAND__STRUCT_H_
#define ATS_MSGS__MSG__DETAIL__PLAN_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'mission_id'
// Member 'plan_json'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/PlanCommand in the package ats_msgs.
typedef struct ats_msgs__msg__PlanCommand
{
  builtin_interfaces__msg__Time stamp;
  rosidl_runtime_c__String mission_id;
  rosidl_runtime_c__String plan_json;
} ats_msgs__msg__PlanCommand;

// Struct for a sequence of ats_msgs__msg__PlanCommand.
typedef struct ats_msgs__msg__PlanCommand__Sequence
{
  ats_msgs__msg__PlanCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ats_msgs__msg__PlanCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ATS_MSGS__MSG__DETAIL__PLAN_COMMAND__STRUCT_H_
