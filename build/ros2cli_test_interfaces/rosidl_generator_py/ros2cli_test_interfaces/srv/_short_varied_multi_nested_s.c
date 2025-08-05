// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ros2cli_test_interfaces:srv/ShortVariedMultiNested.idl
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
#include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__struct.h"
#include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__functions.h"

bool ros2cli_test_interfaces__msg__short_varied_nested__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ros2cli_test_interfaces__msg__short_varied_nested__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ros2cli_test_interfaces__srv__short_varied_multi_nested__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("ros2cli_test_interfaces.srv._short_varied_multi_nested", (char *)PyUnicode_1BYTE_DATA(module_attr), 54) == 0);
    assert(strncmp("ShortVariedMultiNested_Request", (char *)PyUnicode_1BYTE_DATA(name_attr), 30) == 0);

    Py_DECREF(module_attr);
    Py_DECREF(name_attr);
    Py_DECREF(class_attr);
  }
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request * ros_message = _ros_message;
  {  // short_varied_nested
    PyObject * field = PyObject_GetAttrString(_pymsg, "short_varied_nested");
    if (!field) {
      return false;
    }
    if (!ros2cli_test_interfaces__msg__short_varied_nested__convert_from_py(field, &ros_message->short_varied_nested)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros2cli_test_interfaces__srv__short_varied_multi_nested__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ShortVariedMultiNested_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros2cli_test_interfaces.srv._short_varied_multi_nested");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ShortVariedMultiNested_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request * ros_message = (ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request *)raw_ros_message;
  {  // short_varied_nested
    PyObject * field = NULL;
    field = ros2cli_test_interfaces__msg__short_varied_nested__convert_to_py(&ros_message->short_varied_nested);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "short_varied_nested", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__struct.h"
// already included above
// #include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool ros2cli_test_interfaces__srv__short_varied_multi_nested__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("ros2cli_test_interfaces.srv._short_varied_multi_nested", (char *)PyUnicode_1BYTE_DATA(module_attr), 54) == 0);
    assert(strncmp("ShortVariedMultiNested_Response", (char *)PyUnicode_1BYTE_DATA(name_attr), 31) == 0);

    Py_DECREF(module_attr);
    Py_DECREF(name_attr);
    Py_DECREF(class_attr);
  }
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response * ros_message = _ros_message;
  {  // bool_value
    PyObject * field = PyObject_GetAttrString(_pymsg, "bool_value");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->bool_value = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros2cli_test_interfaces__srv__short_varied_multi_nested__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ShortVariedMultiNested_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros2cli_test_interfaces.srv._short_varied_multi_nested");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ShortVariedMultiNested_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response * ros_message = (ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response *)raw_ros_message;
  {  // bool_value
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->bool_value ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bool_value", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__struct.h"
// already included above
// #include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes


// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool service_msgs__msg__service_event_info__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * service_msgs__msg__service_event_info__convert_to_py(void * raw_ros_message);
bool ros2cli_test_interfaces__srv__short_varied_multi_nested__request__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ros2cli_test_interfaces__srv__short_varied_multi_nested__request__convert_to_py(void * raw_ros_message);
bool ros2cli_test_interfaces__srv__short_varied_multi_nested__response__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ros2cli_test_interfaces__srv__short_varied_multi_nested__response__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ros2cli_test_interfaces__srv__short_varied_multi_nested__event__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("ros2cli_test_interfaces.srv._short_varied_multi_nested", (char *)PyUnicode_1BYTE_DATA(module_attr), 54) == 0);
    assert(strncmp("ShortVariedMultiNested_Event", (char *)PyUnicode_1BYTE_DATA(name_attr), 28) == 0);

    Py_DECREF(module_attr);
    Py_DECREF(name_attr);
    Py_DECREF(class_attr);
  }
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event * ros_message = _ros_message;
  {  // info
    PyObject * field = PyObject_GetAttrString(_pymsg, "info");
    if (!field) {
      return false;
    }
    if (!service_msgs__msg__service_event_info__convert_from_py(field, &ros_message->info)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // request
    PyObject * field = PyObject_GetAttrString(_pymsg, "request");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'request'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__Sequence__init(&(ros_message->request), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request * dest = ros_message->request.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!ros2cli_test_interfaces__srv__short_varied_multi_nested__request__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // response
    PyObject * field = PyObject_GetAttrString(_pymsg, "response");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'response'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__Sequence__init(&(ros_message->response), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response * dest = ros_message->response.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!ros2cli_test_interfaces__srv__short_varied_multi_nested__response__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros2cli_test_interfaces__srv__short_varied_multi_nested__event__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ShortVariedMultiNested_Event */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros2cli_test_interfaces.srv._short_varied_multi_nested");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ShortVariedMultiNested_Event");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event * ros_message = (ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event *)raw_ros_message;
  {  // info
    PyObject * field = NULL;
    field = service_msgs__msg__service_event_info__convert_to_py(&ros_message->info);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "info", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // request
    PyObject * field = NULL;
    size_t size = ros_message->request.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->request.data[i]);
      PyObject * pyitem = ros2cli_test_interfaces__srv__short_varied_multi_nested__request__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "request", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // response
    PyObject * field = NULL;
    size_t size = ros_message->response.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->response.data[i]);
      PyObject * pyitem = ros2cli_test_interfaces__srv__short_varied_multi_nested__response__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "response", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
