// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ats_msgs:msg/ReplanRequest.idl
// generated code does not contain a copyright notice

#ifndef ATS_MSGS__MSG__DETAIL__REPLAN_REQUEST__FUNCTIONS_H_
#define ATS_MSGS__MSG__DETAIL__REPLAN_REQUEST__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ats_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ats_msgs/msg/detail/replan_request__struct.h"

/// Initialize msg/ReplanRequest message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ats_msgs__msg__ReplanRequest
 * )) before or use
 * ats_msgs__msg__ReplanRequest__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ats_msgs
bool
ats_msgs__msg__ReplanRequest__init(ats_msgs__msg__ReplanRequest * msg);

/// Finalize msg/ReplanRequest message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ats_msgs
void
ats_msgs__msg__ReplanRequest__fini(ats_msgs__msg__ReplanRequest * msg);

/// Create msg/ReplanRequest message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ats_msgs__msg__ReplanRequest__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ats_msgs
ats_msgs__msg__ReplanRequest *
ats_msgs__msg__ReplanRequest__create();

/// Destroy msg/ReplanRequest message.
/**
 * It calls
 * ats_msgs__msg__ReplanRequest__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ats_msgs
void
ats_msgs__msg__ReplanRequest__destroy(ats_msgs__msg__ReplanRequest * msg);

/// Check for msg/ReplanRequest message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ats_msgs
bool
ats_msgs__msg__ReplanRequest__are_equal(const ats_msgs__msg__ReplanRequest * lhs, const ats_msgs__msg__ReplanRequest * rhs);

/// Copy a msg/ReplanRequest message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ats_msgs
bool
ats_msgs__msg__ReplanRequest__copy(
  const ats_msgs__msg__ReplanRequest * input,
  ats_msgs__msg__ReplanRequest * output);

/// Initialize array of msg/ReplanRequest messages.
/**
 * It allocates the memory for the number of elements and calls
 * ats_msgs__msg__ReplanRequest__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ats_msgs
bool
ats_msgs__msg__ReplanRequest__Sequence__init(ats_msgs__msg__ReplanRequest__Sequence * array, size_t size);

/// Finalize array of msg/ReplanRequest messages.
/**
 * It calls
 * ats_msgs__msg__ReplanRequest__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ats_msgs
void
ats_msgs__msg__ReplanRequest__Sequence__fini(ats_msgs__msg__ReplanRequest__Sequence * array);

/// Create array of msg/ReplanRequest messages.
/**
 * It allocates the memory for the array and calls
 * ats_msgs__msg__ReplanRequest__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ats_msgs
ats_msgs__msg__ReplanRequest__Sequence *
ats_msgs__msg__ReplanRequest__Sequence__create(size_t size);

/// Destroy array of msg/ReplanRequest messages.
/**
 * It calls
 * ats_msgs__msg__ReplanRequest__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ats_msgs
void
ats_msgs__msg__ReplanRequest__Sequence__destroy(ats_msgs__msg__ReplanRequest__Sequence * array);

/// Check for msg/ReplanRequest message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ats_msgs
bool
ats_msgs__msg__ReplanRequest__Sequence__are_equal(const ats_msgs__msg__ReplanRequest__Sequence * lhs, const ats_msgs__msg__ReplanRequest__Sequence * rhs);

/// Copy an array of msg/ReplanRequest messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ats_msgs
bool
ats_msgs__msg__ReplanRequest__Sequence__copy(
  const ats_msgs__msg__ReplanRequest__Sequence * input,
  ats_msgs__msg__ReplanRequest__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ATS_MSGS__MSG__DETAIL__REPLAN_REQUEST__FUNCTIONS_H_
