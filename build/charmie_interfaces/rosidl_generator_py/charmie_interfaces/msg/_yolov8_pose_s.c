// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from charmie_interfaces:msg/Yolov8Pose.idl
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
#include "charmie_interfaces/msg/detail/yolov8_pose__struct.h"
#include "charmie_interfaces/msg/detail/yolov8_pose__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "charmie_interfaces/msg/detail/detected_person__functions.h"
// end nested array functions include
bool charmie_interfaces__msg__detected_person__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * charmie_interfaces__msg__detected_person__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool charmie_interfaces__msg__yolov8_pose__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("charmie_interfaces.msg._yolov8_pose.Yolov8Pose", full_classname_dest, 46) == 0);
  }
  charmie_interfaces__msg__Yolov8Pose * ros_message = _ros_message;
  {  // num_person
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_person");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_person = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // persons
    PyObject * field = PyObject_GetAttrString(_pymsg, "persons");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'persons'");
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
    if (!charmie_interfaces__msg__DetectedPerson__Sequence__init(&(ros_message->persons), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create charmie_interfaces__msg__DetectedPerson__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    charmie_interfaces__msg__DetectedPerson * dest = ros_message->persons.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!charmie_interfaces__msg__detected_person__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
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
PyObject * charmie_interfaces__msg__yolov8_pose__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Yolov8Pose */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("charmie_interfaces.msg._yolov8_pose");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Yolov8Pose");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  charmie_interfaces__msg__Yolov8Pose * ros_message = (charmie_interfaces__msg__Yolov8Pose *)raw_ros_message;
  {  // num_person
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->num_person);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_person", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // persons
    PyObject * field = NULL;
    size_t size = ros_message->persons.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    charmie_interfaces__msg__DetectedPerson * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->persons.data[i]);
      PyObject * pyitem = charmie_interfaces__msg__detected_person__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "persons", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
