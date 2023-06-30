// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from charmie_interfaces:msg/Keypoints.idl
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
#include "charmie_interfaces/msg/detail/keypoints__struct.h"
#include "charmie_interfaces/msg/detail/keypoints__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool charmie_interfaces__msg__keypoints__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[44];
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
    assert(strncmp("charmie_interfaces.msg._keypoints.Keypoints", full_classname_dest, 43) == 0);
  }
  charmie_interfaces__msg__Keypoints * ros_message = _ros_message;
  {  // index_person
    PyObject * field = PyObject_GetAttrString(_pymsg, "index_person");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->index_person = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // average_distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "average_distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->average_distance = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // standard_deviation
    PyObject * field = PyObject_GetAttrString(_pymsg, "standard_deviation");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->standard_deviation = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // key_p0_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p0_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p0_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p0_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p0_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p0_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p1_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p1_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p1_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p1_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p1_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p1_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p2_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p2_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p2_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p2_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p2_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p2_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p3_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p3_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p3_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p3_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p3_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p3_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p4_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p4_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p4_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p4_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p4_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p4_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p5_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p5_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p5_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p5_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p5_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p5_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p6_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p6_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p6_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p6_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p6_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p6_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p7_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p7_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p7_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p7_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p7_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p7_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p8_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p8_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p8_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p8_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p8_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p8_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p9_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p9_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p9_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p9_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p9_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p9_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p10_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p10_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p10_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p10_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p10_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p10_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p11_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p11_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p11_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p11_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p11_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p11_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p12_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p12_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p12_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p12_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p12_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p12_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p13_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p13_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p13_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p13_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p13_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p13_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p14_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p14_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p14_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p14_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p14_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p14_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p15_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p15_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p15_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p15_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p15_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p15_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p16_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p16_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p16_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_p16_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_p16_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_p16_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * charmie_interfaces__msg__keypoints__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Keypoints */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("charmie_interfaces.msg._keypoints");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Keypoints");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  charmie_interfaces__msg__Keypoints * ros_message = (charmie_interfaces__msg__Keypoints *)raw_ros_message;
  {  // index_person
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->index_person);
    {
      int rc = PyObject_SetAttrString(_pymessage, "index_person", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // average_distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->average_distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "average_distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // standard_deviation
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->standard_deviation);
    {
      int rc = PyObject_SetAttrString(_pymessage, "standard_deviation", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p0_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p0_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p0_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p0_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p0_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p0_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p1_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p1_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p1_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p1_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p1_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p1_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p2_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p2_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p2_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p2_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p2_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p2_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p3_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p3_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p3_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p3_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p3_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p3_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p4_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p4_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p4_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p4_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p4_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p4_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p5_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p5_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p5_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p5_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p5_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p5_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p6_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p6_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p6_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p6_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p6_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p6_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p7_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p7_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p7_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p7_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p7_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p7_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p8_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p8_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p8_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p8_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p8_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p8_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p9_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p9_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p9_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p9_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p9_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p9_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p10_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p10_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p10_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p10_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p10_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p10_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p11_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p11_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p11_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p11_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p11_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p11_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p12_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p12_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p12_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p12_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p12_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p12_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p13_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p13_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p13_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p13_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p13_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p13_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p14_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p14_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p14_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p14_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p14_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p14_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p15_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p15_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p15_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p15_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p15_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p15_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p16_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p16_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p16_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_p16_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_p16_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_p16_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
