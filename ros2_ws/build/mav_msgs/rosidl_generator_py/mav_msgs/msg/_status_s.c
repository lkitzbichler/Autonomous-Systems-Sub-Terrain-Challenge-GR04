// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mav_msgs:msg/Status.idl
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
#include "mav_msgs/msg/detail/status__struct.h"
#include "mav_msgs/msg/detail/status__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mav_msgs__msg__status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[28];
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
    assert(strncmp("mav_msgs.msg._status.Status", full_classname_dest, 27) == 0);
  }
  mav_msgs__msg__Status * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // vehicle_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "vehicle_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->vehicle_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // vehicle_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "vehicle_type");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->vehicle_type, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // battery_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->battery_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rc_command_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "rc_command_mode");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->rc_command_mode, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // command_interface_enabled
    PyObject * field = PyObject_GetAttrString(_pymsg, "command_interface_enabled");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->command_interface_enabled = (Py_True == field);
    Py_DECREF(field);
  }
  {  // flight_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "flight_time");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->flight_time = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // system_uptime
    PyObject * field = PyObject_GetAttrString(_pymsg, "system_uptime");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->system_uptime = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cpu_load
    PyObject * field = PyObject_GetAttrString(_pymsg, "cpu_load");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cpu_load = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // motor_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_status");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->motor_status, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // in_air
    PyObject * field = PyObject_GetAttrString(_pymsg, "in_air");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->in_air = (Py_True == field);
    Py_DECREF(field);
  }
  {  // gps_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "gps_status");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->gps_status, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // gps_num_satellites
    PyObject * field = PyObject_GetAttrString(_pymsg, "gps_num_satellites");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->gps_num_satellites = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mav_msgs__msg__status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Status */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mav_msgs.msg._status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Status");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mav_msgs__msg__Status * ros_message = (mav_msgs__msg__Status *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vehicle_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->vehicle_name.data,
      strlen(ros_message->vehicle_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "vehicle_name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vehicle_type
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->vehicle_type.data,
      strlen(ros_message->vehicle_type.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "vehicle_type", field);
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
  {  // rc_command_mode
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->rc_command_mode.data,
      strlen(ros_message->rc_command_mode.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "rc_command_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // command_interface_enabled
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->command_interface_enabled ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "command_interface_enabled", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // flight_time
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->flight_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "flight_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // system_uptime
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->system_uptime);
    {
      int rc = PyObject_SetAttrString(_pymessage, "system_uptime", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cpu_load
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cpu_load);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cpu_load", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_status
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->motor_status.data,
      strlen(ros_message->motor_status.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // in_air
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->in_air ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "in_air", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gps_status
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->gps_status.data,
      strlen(ros_message->gps_status.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "gps_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gps_num_satellites
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->gps_num_satellites);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gps_num_satellites", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
