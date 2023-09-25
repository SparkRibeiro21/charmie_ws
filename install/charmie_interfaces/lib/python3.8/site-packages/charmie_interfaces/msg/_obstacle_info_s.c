// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from charmie_interfaces:msg/ObstacleInfo.idl
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
#include "charmie_interfaces/msg/detail/obstacle_info__struct.h"
#include "charmie_interfaces/msg/detail/obstacle_info__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool charmie_interfaces__msg__obstacle_info__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[51];
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
    assert(strncmp("charmie_interfaces.msg._obstacle_info.ObstacleInfo", full_classname_dest, 50) == 0);
  }
  charmie_interfaces__msg__ObstacleInfo * ros_message = _ros_message;
  {  // alfa
    PyObject * field = PyObject_GetAttrString(_pymsg, "alfa");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->alfa = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // dist
    PyObject * field = PyObject_GetAttrString(_pymsg, "dist");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->dist = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // length_cm
    PyObject * field = PyObject_GetAttrString(_pymsg, "length_cm");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->length_cm = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // length_degrees
    PyObject * field = PyObject_GetAttrString(_pymsg, "length_degrees");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->length_degrees = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * charmie_interfaces__msg__obstacle_info__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ObstacleInfo */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("charmie_interfaces.msg._obstacle_info");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ObstacleInfo");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  charmie_interfaces__msg__ObstacleInfo * ros_message = (charmie_interfaces__msg__ObstacleInfo *)raw_ros_message;
  {  // alfa
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->alfa);
    {
      int rc = PyObject_SetAttrString(_pymessage, "alfa", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dist
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->dist);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dist", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // length_cm
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->length_cm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "length_cm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // length_degrees
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->length_degrees);
    {
      int rc = PyObject_SetAttrString(_pymessage, "length_degrees", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
