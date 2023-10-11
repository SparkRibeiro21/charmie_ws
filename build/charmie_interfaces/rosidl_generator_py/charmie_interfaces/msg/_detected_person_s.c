// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from charmie_interfaces:msg/DetectedPerson.idl
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
#include "charmie_interfaces/msg/detail/detected_person__struct.h"
#include "charmie_interfaces/msg/detail/detected_person__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool charmie_interfaces__msg__detected_person__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[55];
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
    assert(strncmp("charmie_interfaces.msg._detected_person.DetectedPerson", full_classname_dest, 54) == 0);
  }
  charmie_interfaces__msg__DetectedPerson * ros_message = _ros_message;
  {  // index_person
    PyObject * field = PyObject_GetAttrString(_pymsg, "index_person");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->index_person = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // conf_person
    PyObject * field = PyObject_GetAttrString(_pymsg, "conf_person");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->conf_person = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // x_rel
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_rel");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_rel = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_rel
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_rel");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_rel = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // box_top_left_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "box_top_left_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->box_top_left_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // box_top_left_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "box_top_left_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->box_top_left_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // box_width
    PyObject * field = PyObject_GetAttrString(_pymsg, "box_width");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->box_width = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // box_height
    PyObject * field = PyObject_GetAttrString(_pymsg, "box_height");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->box_height = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_nose_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_nose_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_nose_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_nose_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_nose_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_nose_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_nose_conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_nose_conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp_nose_conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kp_eye_left_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_eye_left_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_eye_left_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_eye_left_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_eye_left_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_eye_left_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_eye_left_conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_eye_left_conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp_eye_left_conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kp_eye_right_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_eye_right_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_eye_right_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_eye_right_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_eye_right_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_eye_right_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_eye_right_conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_eye_right_conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp_eye_right_conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kp_ear_left_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_ear_left_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_ear_left_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_ear_left_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_ear_left_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_ear_left_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_ear_left_conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_ear_left_conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp_ear_left_conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kp_ear_right_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_ear_right_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_ear_right_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_ear_right_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_ear_right_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_ear_right_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_ear_right_conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_ear_right_conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp_ear_right_conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kp_shoulder_left_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_shoulder_left_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_shoulder_left_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_shoulder_left_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_shoulder_left_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_shoulder_left_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_shoulder_left_conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_shoulder_left_conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp_shoulder_left_conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kp_shoulder_right_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_shoulder_right_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_shoulder_right_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_shoulder_right_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_shoulder_right_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_shoulder_right_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_shoulder_right_conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_shoulder_right_conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp_shoulder_right_conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kp_elbow_left_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_elbow_left_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_elbow_left_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_elbow_left_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_elbow_left_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_elbow_left_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_elbow_left_conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_elbow_left_conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp_elbow_left_conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kp_elbow_right_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_elbow_right_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_elbow_right_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_elbow_right_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_elbow_right_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_elbow_right_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_elbow_right_conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_elbow_right_conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp_elbow_right_conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kp_wrist_left_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_wrist_left_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_wrist_left_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_wrist_left_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_wrist_left_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_wrist_left_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_wrist_left_conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_wrist_left_conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp_wrist_left_conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kp_wrist_right_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_wrist_right_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_wrist_right_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_wrist_right_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_wrist_right_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_wrist_right_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_wrist_right_conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_wrist_right_conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp_wrist_right_conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kp_hip_left_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_hip_left_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_hip_left_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_hip_left_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_hip_left_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_hip_left_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_hip_left_conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_hip_left_conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp_hip_left_conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kp_hip_right_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_hip_right_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_hip_right_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_hip_right_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_hip_right_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_hip_right_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_hip_right_conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_hip_right_conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp_hip_right_conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kp_knee_left_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_knee_left_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_knee_left_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_knee_left_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_knee_left_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_knee_left_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_knee_left_conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_knee_left_conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp_knee_left_conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kp_knee_right_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_knee_right_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_knee_right_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_knee_right_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_knee_right_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_knee_right_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_knee_right_conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_knee_right_conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp_knee_right_conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kp_ankle_left_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_ankle_left_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_ankle_left_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_ankle_left_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_ankle_left_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_ankle_left_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_ankle_left_conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_ankle_left_conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp_ankle_left_conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kp_ankle_right_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_ankle_right_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_ankle_right_x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_ankle_right_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_ankle_right_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->kp_ankle_right_y = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // kp_ankle_right_conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "kp_ankle_right_conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kp_ankle_right_conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * charmie_interfaces__msg__detected_person__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DetectedPerson */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("charmie_interfaces.msg._detected_person");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DetectedPerson");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  charmie_interfaces__msg__DetectedPerson * ros_message = (charmie_interfaces__msg__DetectedPerson *)raw_ros_message;
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
  {  // conf_person
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->conf_person);
    {
      int rc = PyObject_SetAttrString(_pymessage, "conf_person", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x_rel
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_rel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_rel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_rel
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_rel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_rel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // box_top_left_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->box_top_left_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "box_top_left_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // box_top_left_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->box_top_left_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "box_top_left_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // box_width
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->box_width);
    {
      int rc = PyObject_SetAttrString(_pymessage, "box_width", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // box_height
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->box_height);
    {
      int rc = PyObject_SetAttrString(_pymessage, "box_height", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_nose_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_nose_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_nose_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_nose_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_nose_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_nose_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_nose_conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp_nose_conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_nose_conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_eye_left_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_eye_left_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_eye_left_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_eye_left_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_eye_left_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_eye_left_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_eye_left_conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp_eye_left_conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_eye_left_conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_eye_right_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_eye_right_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_eye_right_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_eye_right_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_eye_right_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_eye_right_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_eye_right_conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp_eye_right_conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_eye_right_conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_ear_left_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_ear_left_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_ear_left_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_ear_left_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_ear_left_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_ear_left_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_ear_left_conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp_ear_left_conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_ear_left_conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_ear_right_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_ear_right_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_ear_right_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_ear_right_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_ear_right_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_ear_right_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_ear_right_conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp_ear_right_conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_ear_right_conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_shoulder_left_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_shoulder_left_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_shoulder_left_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_shoulder_left_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_shoulder_left_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_shoulder_left_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_shoulder_left_conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp_shoulder_left_conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_shoulder_left_conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_shoulder_right_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_shoulder_right_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_shoulder_right_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_shoulder_right_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_shoulder_right_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_shoulder_right_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_shoulder_right_conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp_shoulder_right_conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_shoulder_right_conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_elbow_left_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_elbow_left_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_elbow_left_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_elbow_left_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_elbow_left_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_elbow_left_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_elbow_left_conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp_elbow_left_conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_elbow_left_conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_elbow_right_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_elbow_right_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_elbow_right_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_elbow_right_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_elbow_right_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_elbow_right_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_elbow_right_conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp_elbow_right_conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_elbow_right_conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_wrist_left_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_wrist_left_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_wrist_left_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_wrist_left_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_wrist_left_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_wrist_left_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_wrist_left_conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp_wrist_left_conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_wrist_left_conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_wrist_right_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_wrist_right_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_wrist_right_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_wrist_right_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_wrist_right_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_wrist_right_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_wrist_right_conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp_wrist_right_conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_wrist_right_conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_hip_left_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_hip_left_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_hip_left_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_hip_left_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_hip_left_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_hip_left_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_hip_left_conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp_hip_left_conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_hip_left_conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_hip_right_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_hip_right_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_hip_right_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_hip_right_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_hip_right_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_hip_right_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_hip_right_conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp_hip_right_conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_hip_right_conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_knee_left_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_knee_left_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_knee_left_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_knee_left_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_knee_left_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_knee_left_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_knee_left_conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp_knee_left_conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_knee_left_conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_knee_right_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_knee_right_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_knee_right_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_knee_right_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_knee_right_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_knee_right_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_knee_right_conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp_knee_right_conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_knee_right_conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_ankle_left_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_ankle_left_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_ankle_left_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_ankle_left_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_ankle_left_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_ankle_left_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_ankle_left_conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp_ankle_left_conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_ankle_left_conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_ankle_right_x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_ankle_right_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_ankle_right_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_ankle_right_y
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->kp_ankle_right_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_ankle_right_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kp_ankle_right_conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kp_ankle_right_conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kp_ankle_right_conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
