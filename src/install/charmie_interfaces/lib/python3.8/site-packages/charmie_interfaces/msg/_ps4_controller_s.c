// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from charmie_interfaces:msg/PS4Controller.idl
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
#include "charmie_interfaces/msg/detail/ps4_controller__struct.h"
#include "charmie_interfaces/msg/detail/ps4_controller__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool charmie_interfaces__msg__ps4_controller__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[53];
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
    assert(strncmp("charmie_interfaces.msg._ps4_controller.PS4Controller", full_classname_dest, 52) == 0);
  }
  charmie_interfaces__msg__PS4Controller * ros_message = _ros_message;
  {  // triangle
    PyObject * field = PyObject_GetAttrString(_pymsg, "triangle");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->triangle = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // circle
    PyObject * field = PyObject_GetAttrString(_pymsg, "circle");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->circle = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // cross
    PyObject * field = PyObject_GetAttrString(_pymsg, "cross");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->cross = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // square
    PyObject * field = PyObject_GetAttrString(_pymsg, "square");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->square = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // arrow_up
    PyObject * field = PyObject_GetAttrString(_pymsg, "arrow_up");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->arrow_up = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // arrow_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "arrow_right");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->arrow_right = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // arrow_down
    PyObject * field = PyObject_GetAttrString(_pymsg, "arrow_down");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->arrow_down = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // arrow_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "arrow_left");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->arrow_left = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // l1
    PyObject * field = PyObject_GetAttrString(_pymsg, "l1");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->l1 = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // r1
    PyObject * field = PyObject_GetAttrString(_pymsg, "r1");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->r1 = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // l3
    PyObject * field = PyObject_GetAttrString(_pymsg, "l3");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->l3 = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // r3
    PyObject * field = PyObject_GetAttrString(_pymsg, "r3");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->r3 = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // share
    PyObject * field = PyObject_GetAttrString(_pymsg, "share");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->share = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // options
    PyObject * field = PyObject_GetAttrString(_pymsg, "options");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->options = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // ps
    PyObject * field = PyObject_GetAttrString(_pymsg, "ps");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ps = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // l3_ang
    PyObject * field = PyObject_GetAttrString(_pymsg, "l3_ang");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->l3_ang = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // l3_dist
    PyObject * field = PyObject_GetAttrString(_pymsg, "l3_dist");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->l3_dist = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // l3_xx
    PyObject * field = PyObject_GetAttrString(_pymsg, "l3_xx");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->l3_xx = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // l3_yy
    PyObject * field = PyObject_GetAttrString(_pymsg, "l3_yy");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->l3_yy = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // r3_ang
    PyObject * field = PyObject_GetAttrString(_pymsg, "r3_ang");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->r3_ang = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // r3_dist
    PyObject * field = PyObject_GetAttrString(_pymsg, "r3_dist");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->r3_dist = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // r3_xx
    PyObject * field = PyObject_GetAttrString(_pymsg, "r3_xx");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->r3_xx = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // r3_yy
    PyObject * field = PyObject_GetAttrString(_pymsg, "r3_yy");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->r3_yy = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // l2
    PyObject * field = PyObject_GetAttrString(_pymsg, "l2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->l2 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // r2
    PyObject * field = PyObject_GetAttrString(_pymsg, "r2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->r2 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * charmie_interfaces__msg__ps4_controller__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PS4Controller */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("charmie_interfaces.msg._ps4_controller");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PS4Controller");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  charmie_interfaces__msg__PS4Controller * ros_message = (charmie_interfaces__msg__PS4Controller *)raw_ros_message;
  {  // triangle
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->triangle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "triangle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // circle
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->circle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "circle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cross
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->cross);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cross", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // square
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->square);
    {
      int rc = PyObject_SetAttrString(_pymessage, "square", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // arrow_up
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->arrow_up);
    {
      int rc = PyObject_SetAttrString(_pymessage, "arrow_up", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // arrow_right
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->arrow_right);
    {
      int rc = PyObject_SetAttrString(_pymessage, "arrow_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // arrow_down
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->arrow_down);
    {
      int rc = PyObject_SetAttrString(_pymessage, "arrow_down", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // arrow_left
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->arrow_left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "arrow_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // l1
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->l1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "l1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // r1
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->r1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "r1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // l3
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->l3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "l3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // r3
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->r3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "r3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // share
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->share);
    {
      int rc = PyObject_SetAttrString(_pymessage, "share", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // options
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->options);
    {
      int rc = PyObject_SetAttrString(_pymessage, "options", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ps
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->ps);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // l3_ang
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->l3_ang);
    {
      int rc = PyObject_SetAttrString(_pymessage, "l3_ang", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // l3_dist
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->l3_dist);
    {
      int rc = PyObject_SetAttrString(_pymessage, "l3_dist", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // l3_xx
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->l3_xx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "l3_xx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // l3_yy
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->l3_yy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "l3_yy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // r3_ang
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->r3_ang);
    {
      int rc = PyObject_SetAttrString(_pymessage, "r3_ang", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // r3_dist
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->r3_dist);
    {
      int rc = PyObject_SetAttrString(_pymessage, "r3_dist", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // r3_xx
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->r3_xx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "r3_xx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // r3_yy
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->r3_yy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "r3_yy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // l2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->l2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "l2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // r2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->r2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "r2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
