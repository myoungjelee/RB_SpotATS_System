// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ats_msgs:msg/AtsState.idl
// generated code does not contain a copyright notice

#ifndef ATS_MSGS__MSG__DETAIL__ATS_STATE__STRUCT_H_
#define ATS_MSGS__MSG__DETAIL__ATS_STATE__STRUCT_H_

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
// Member 'system_state'
// Member 'pose_frame'
// Member 'vision_json'
// Member 'plan_json'
// Member 'queue_status'
// Member 'history_json'
// Member 'last_violation'
// Member 'events_json'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/AtsState in the package ats_msgs.
typedef struct ats_msgs__msg__AtsState
{
  builtin_interfaces__msg__Time stamp;
  rosidl_runtime_c__String system_state;
  rosidl_runtime_c__String pose_frame;
  double pose_x;
  double pose_y;
  double pose_yaw;
  double vel_vx;
  double vel_vy;
  double vel_wz;
  double battery_soc;
  double battery_voltage;
  int32_t primary_id;
  double lost_sec;
  rosidl_runtime_c__String vision_json;
  rosidl_runtime_c__String plan_json;
  int32_t current_index;
  rosidl_runtime_c__String queue_status;
  rosidl_runtime_c__String history_json;
  bool roe_ok;
  bool safe_backstop;
  double max_speed;
  rosidl_runtime_c__String last_violation;
  rosidl_runtime_c__String events_json;
} ats_msgs__msg__AtsState;

// Struct for a sequence of ats_msgs__msg__AtsState.
typedef struct ats_msgs__msg__AtsState__Sequence
{
  ats_msgs__msg__AtsState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ats_msgs__msg__AtsState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ATS_MSGS__MSG__DETAIL__ATS_STATE__STRUCT_H_
