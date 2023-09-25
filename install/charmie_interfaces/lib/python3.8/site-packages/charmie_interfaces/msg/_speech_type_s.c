// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from charmie_interfaces:msg/SpeechType.idl
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
#include "charmie_interfaces/msg/detail/speech_type__struct.h"
#include "charmie_interfaces/msg/detail/speech_type__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool charmie_interfaces__msg__speech_type__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[47];
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
    assert(strncmp("charmie_interfaces.msg._speech_type.SpeechType", full_classname_dest, 46) == 0);
  }
  charmie_interfaces__msg__SpeechType * ros_message = _ros_message;
  {  // yes_or_no
    PyObject * field = PyObject_GetAttrString(_pymsg, "yes_or_no");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->yes_or_no = (Py_True == field);
    Py_DECREF(field);
  }
  {  // receptionist
    PyObject * field = PyObject_GetAttrString(_pymsg, "receptionist");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->receptionist = (Py_True == field);
    Py_DECREF(field);
  }
  {  // gpsr
    PyObject * field = PyObject_GetAttrString(_pymsg, "gpsr");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->gpsr = (Py_True == field);
    Py_DECREF(field);
  }
  {  // restaurant
    PyObject * field = PyObject_GetAttrString(_pymsg, "restaurant");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->restaurant = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * charmie_interfaces__msg__speech_type__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SpeechType */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("charmie_interfaces.msg._speech_type");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SpeechType");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  charmie_interfaces__msg__SpeechType * ros_message = (charmie_interfaces__msg__SpeechType *)raw_ros_message;
  {  // yes_or_no
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->yes_or_no ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yes_or_no", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // receptionist
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->receptionist ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "receptionist", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gpsr
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->gpsr ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gpsr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // restaurant
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->restaurant ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "restaurant", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
