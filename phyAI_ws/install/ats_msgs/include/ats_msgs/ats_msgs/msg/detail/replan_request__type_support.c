// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ats_msgs:msg/ReplanRequest.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ats_msgs/msg/detail/replan_request__rosidl_typesupport_introspection_c.h"
#include "ats_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ats_msgs/msg/detail/replan_request__functions.h"
#include "ats_msgs/msg/detail/replan_request__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `mission_id`
// Member `reason`
// Member `context_json`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ats_msgs__msg__ReplanRequest__rosidl_typesupport_introspection_c__ReplanRequest_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ats_msgs__msg__ReplanRequest__init(message_memory);
}

void ats_msgs__msg__ReplanRequest__rosidl_typesupport_introspection_c__ReplanRequest_fini_function(void * message_memory)
{
  ats_msgs__msg__ReplanRequest__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ats_msgs__msg__ReplanRequest__rosidl_typesupport_introspection_c__ReplanRequest_message_member_array[4] = {
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ats_msgs__msg__ReplanRequest, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mission_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ats_msgs__msg__ReplanRequest, mission_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reason",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ats_msgs__msg__ReplanRequest, reason),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "context_json",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ats_msgs__msg__ReplanRequest, context_json),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ats_msgs__msg__ReplanRequest__rosidl_typesupport_introspection_c__ReplanRequest_message_members = {
  "ats_msgs__msg",  // message namespace
  "ReplanRequest",  // message name
  4,  // number of fields
  sizeof(ats_msgs__msg__ReplanRequest),
  ats_msgs__msg__ReplanRequest__rosidl_typesupport_introspection_c__ReplanRequest_message_member_array,  // message members
  ats_msgs__msg__ReplanRequest__rosidl_typesupport_introspection_c__ReplanRequest_init_function,  // function to initialize message memory (memory has to be allocated)
  ats_msgs__msg__ReplanRequest__rosidl_typesupport_introspection_c__ReplanRequest_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ats_msgs__msg__ReplanRequest__rosidl_typesupport_introspection_c__ReplanRequest_message_type_support_handle = {
  0,
  &ats_msgs__msg__ReplanRequest__rosidl_typesupport_introspection_c__ReplanRequest_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ats_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ats_msgs, msg, ReplanRequest)() {
  ats_msgs__msg__ReplanRequest__rosidl_typesupport_introspection_c__ReplanRequest_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!ats_msgs__msg__ReplanRequest__rosidl_typesupport_introspection_c__ReplanRequest_message_type_support_handle.typesupport_identifier) {
    ats_msgs__msg__ReplanRequest__rosidl_typesupport_introspection_c__ReplanRequest_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ats_msgs__msg__ReplanRequest__rosidl_typesupport_introspection_c__ReplanRequest_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
