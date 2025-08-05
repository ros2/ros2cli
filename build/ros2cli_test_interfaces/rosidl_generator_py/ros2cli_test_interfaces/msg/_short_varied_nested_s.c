// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ros2cli_test_interfaces:msg/ShortVariedNested.idl
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
#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__struct.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__functions.h"

bool ros2cli_test_interfaces__msg__short_varied__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ros2cli_test_interfaces__msg__short_varied__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ros2cli_test_interfaces__msg__short_varied_nested__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
    if (class_attr == NULL) {
      return false;
    }
    PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
    if (name_attr == NULL) {
      Py_DECREF(class_attr);
      return false;
    }
    PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
    if (module_attr == NULL) {
      Py_DECREF(name_attr);
      Py_DECREF(class_attr);
      return false;
    }

    // PyUnicode_1BYTE_DATA is just a cast
    assert(strncmp("ros2cli_test_interfaces.msg._short_varied_nested", (char *)PyUnicode_1BYTE_DATA(module_attr), 48) == 0);
    assert(strncmp("ShortVariedNested", (char *)PyUnicode_1BYTE_DATA(name_attr), 17) == 0);

    Py_DECREF(module_attr);
    Py_DECREF(name_attr);
    Py_DECREF(class_attr);
  }
  ros2cli_test_interfaces__msg__ShortVariedNested * ros_message = _ros_message;
  {  // short_varied
    PyObject * field = PyObject_GetAttrString(_pymsg, "short_varied");
    if (!field) {
      return false;
    }
    if (!ros2cli_test_interfaces__msg__short_varied__convert_from_py(field, &ros_message->short_varied)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros2cli_test_interfaces__msg__short_varied_nested__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ShortVariedNested */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros2cli_test_interfaces.msg._short_varied_nested");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ShortVariedNested");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros2cli_test_interfaces__msg__ShortVariedNested * ros_message = (ros2cli_test_interfaces__msg__ShortVariedNested *)raw_ros_message;
  {  // short_varied
    PyObject * field = NULL;
    field = ros2cli_test_interfaces__msg__short_varied__convert_to_py(&ros_message->short_varied);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "short_varied", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
