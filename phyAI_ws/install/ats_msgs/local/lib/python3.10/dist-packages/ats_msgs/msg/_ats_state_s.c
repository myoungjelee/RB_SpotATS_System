// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ats_msgs:msg/AtsState.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "ats_msgs/msg/detail/ats_state__struct.h"
#include "ats_msgs/msg/detail/ats_state__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ats_msgs__msg__ats_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[33];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("ats_msgs.msg._ats_state.AtsState", full_classname_dest, 32) == 0);
  }
  ats_msgs__msg__AtsState * ros_message = _ros_message;
  {  // stamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "stamp");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->stamp)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // system_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "system_state");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->system_state, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // pose_frame
    PyObject * field = PyObject_GetAttrString(_pymsg, "pose_frame");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->pose_frame, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // pose_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "pose_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pose_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pose_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "pose_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pose_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pose_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "pose_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pose_yaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vel_vx
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_vx");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vel_vx = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vel_vy
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_vy");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vel_vy = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vel_wz
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_wz");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vel_wz = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // battery_soc
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery_soc");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->battery_soc = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // battery_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->battery_voltage = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // primary_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "primary_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->primary_id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // lost_sec
    PyObject * field = PyObject_GetAttrString(_pymsg, "lost_sec");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->lost_sec = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vision_json
    PyObject * field = PyObject_GetAttrString(_pymsg, "vision_json");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->vision_json, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // plan_json
    PyObject * field = PyObject_GetAttrString(_pymsg, "plan_json");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->plan_json, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // current_index
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_index");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->current_index = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // queue_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "queue_status");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->queue_status, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // history_json
    PyObject * field = PyObject_GetAttrString(_pymsg, "history_json");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->history_json, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // roe_ok
    PyObject * field = PyObject_GetAttrString(_pymsg, "roe_ok");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->roe_ok = (Py_True == field);
    Py_DECREF(field);
  }
  {  // safe_backstop
    PyObject * field = PyObject_GetAttrString(_pymsg, "safe_backstop");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->safe_backstop = (Py_True == field);
    Py_DECREF(field);
  }
  {  // max_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_speed = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // last_violation
    PyObject * field = PyObject_GetAttrString(_pymsg, "last_violation");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->last_violation, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // events_json
    PyObject * field = PyObject_GetAttrString(_pymsg, "events_json");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->events_json, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ats_msgs__msg__ats_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of AtsState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ats_msgs.msg._ats_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "AtsState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ats_msgs__msg__AtsState * ros_message = (ats_msgs__msg__AtsState *)raw_ros_message;
  {  // stamp
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->stamp);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "stamp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // system_state
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->system_state.data,
      strlen(ros_message->system_state.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "system_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pose_frame
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->pose_frame.data,
      strlen(ros_message->pose_frame.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "pose_frame", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pose_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pose_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pose_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pose_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pose_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pose_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pose_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pose_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pose_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_vx
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vel_vx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_vx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_vy
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vel_vy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_vy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_wz
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vel_wz);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_wz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // battery_soc
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->battery_soc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery_soc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // battery_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->battery_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // primary_id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->primary_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "primary_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lost_sec
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->lost_sec);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lost_sec", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vision_json
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->vision_json.data,
      strlen(ros_message->vision_json.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "vision_json", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // plan_json
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->plan_json.data,
      strlen(ros_message->plan_json.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "plan_json", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_index
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->current_index);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_index", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // queue_status
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->queue_status.data,
      strlen(ros_message->queue_status.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "queue_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // history_json
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->history_json.data,
      strlen(ros_message->history_json.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "history_json", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // roe_ok
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->roe_ok ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roe_ok", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // safe_backstop
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->safe_backstop ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "safe_backstop", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // last_violation
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->last_violation.data,
      strlen(ros_message->last_violation.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "last_violation", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // events_json
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->events_json.data,
      strlen(ros_message->events_json.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "events_json", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
