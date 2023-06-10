// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from realsense2_camera_msgs:srv/DeviceInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "realsense2_camera_msgs/srv/detail/device_info__rosidl_typesupport_introspection_c.h"
#include "realsense2_camera_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "realsense2_camera_msgs/srv/detail/device_info__functions.h"
#include "realsense2_camera_msgs/srv/detail/device_info__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void DeviceInfo_Request__rosidl_typesupport_introspection_c__DeviceInfo_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  realsense2_camera_msgs__srv__DeviceInfo_Request__init(message_memory);
}

void DeviceInfo_Request__rosidl_typesupport_introspection_c__DeviceInfo_Request_fini_function(void * message_memory)
{
  realsense2_camera_msgs__srv__DeviceInfo_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember DeviceInfo_Request__rosidl_typesupport_introspection_c__DeviceInfo_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(realsense2_camera_msgs__srv__DeviceInfo_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers DeviceInfo_Request__rosidl_typesupport_introspection_c__DeviceInfo_Request_message_members = {
  "realsense2_camera_msgs__srv",  // message namespace
  "DeviceInfo_Request",  // message name
  1,  // number of fields
  sizeof(realsense2_camera_msgs__srv__DeviceInfo_Request),
  DeviceInfo_Request__rosidl_typesupport_introspection_c__DeviceInfo_Request_message_member_array,  // message members
  DeviceInfo_Request__rosidl_typesupport_introspection_c__DeviceInfo_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  DeviceInfo_Request__rosidl_typesupport_introspection_c__DeviceInfo_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t DeviceInfo_Request__rosidl_typesupport_introspection_c__DeviceInfo_Request_message_type_support_handle = {
  0,
  &DeviceInfo_Request__rosidl_typesupport_introspection_c__DeviceInfo_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_realsense2_camera_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, realsense2_camera_msgs, srv, DeviceInfo_Request)() {
  if (!DeviceInfo_Request__rosidl_typesupport_introspection_c__DeviceInfo_Request_message_type_support_handle.typesupport_identifier) {
    DeviceInfo_Request__rosidl_typesupport_introspection_c__DeviceInfo_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &DeviceInfo_Request__rosidl_typesupport_introspection_c__DeviceInfo_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "realsense2_camera_msgs/srv/detail/device_info__rosidl_typesupport_introspection_c.h"
// already included above
// #include "realsense2_camera_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "realsense2_camera_msgs/srv/detail/device_info__functions.h"
// already included above
// #include "realsense2_camera_msgs/srv/detail/device_info__struct.h"


// Include directives for member types
// Member `device_name`
// Member `serial_number`
// Member `firmware_version`
// Member `usb_type_descriptor`
// Member `firmware_update_id`
// Member `sensors`
// Member `physical_port`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void DeviceInfo_Response__rosidl_typesupport_introspection_c__DeviceInfo_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  realsense2_camera_msgs__srv__DeviceInfo_Response__init(message_memory);
}

void DeviceInfo_Response__rosidl_typesupport_introspection_c__DeviceInfo_Response_fini_function(void * message_memory)
{
  realsense2_camera_msgs__srv__DeviceInfo_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember DeviceInfo_Response__rosidl_typesupport_introspection_c__DeviceInfo_Response_message_member_array[7] = {
  {
    "device_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(realsense2_camera_msgs__srv__DeviceInfo_Response, device_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "serial_number",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(realsense2_camera_msgs__srv__DeviceInfo_Response, serial_number),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "firmware_version",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(realsense2_camera_msgs__srv__DeviceInfo_Response, firmware_version),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "usb_type_descriptor",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(realsense2_camera_msgs__srv__DeviceInfo_Response, usb_type_descriptor),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "firmware_update_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(realsense2_camera_msgs__srv__DeviceInfo_Response, firmware_update_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sensors",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(realsense2_camera_msgs__srv__DeviceInfo_Response, sensors),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "physical_port",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(realsense2_camera_msgs__srv__DeviceInfo_Response, physical_port),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers DeviceInfo_Response__rosidl_typesupport_introspection_c__DeviceInfo_Response_message_members = {
  "realsense2_camera_msgs__srv",  // message namespace
  "DeviceInfo_Response",  // message name
  7,  // number of fields
  sizeof(realsense2_camera_msgs__srv__DeviceInfo_Response),
  DeviceInfo_Response__rosidl_typesupport_introspection_c__DeviceInfo_Response_message_member_array,  // message members
  DeviceInfo_Response__rosidl_typesupport_introspection_c__DeviceInfo_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  DeviceInfo_Response__rosidl_typesupport_introspection_c__DeviceInfo_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t DeviceInfo_Response__rosidl_typesupport_introspection_c__DeviceInfo_Response_message_type_support_handle = {
  0,
  &DeviceInfo_Response__rosidl_typesupport_introspection_c__DeviceInfo_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_realsense2_camera_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, realsense2_camera_msgs, srv, DeviceInfo_Response)() {
  if (!DeviceInfo_Response__rosidl_typesupport_introspection_c__DeviceInfo_Response_message_type_support_handle.typesupport_identifier) {
    DeviceInfo_Response__rosidl_typesupport_introspection_c__DeviceInfo_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &DeviceInfo_Response__rosidl_typesupport_introspection_c__DeviceInfo_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "realsense2_camera_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "realsense2_camera_msgs/srv/detail/device_info__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers realsense2_camera_msgs__srv__detail__device_info__rosidl_typesupport_introspection_c__DeviceInfo_service_members = {
  "realsense2_camera_msgs__srv",  // service namespace
  "DeviceInfo",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // realsense2_camera_msgs__srv__detail__device_info__rosidl_typesupport_introspection_c__DeviceInfo_Request_message_type_support_handle,
  NULL  // response message
  // realsense2_camera_msgs__srv__detail__device_info__rosidl_typesupport_introspection_c__DeviceInfo_Response_message_type_support_handle
};

static rosidl_service_type_support_t realsense2_camera_msgs__srv__detail__device_info__rosidl_typesupport_introspection_c__DeviceInfo_service_type_support_handle = {
  0,
  &realsense2_camera_msgs__srv__detail__device_info__rosidl_typesupport_introspection_c__DeviceInfo_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, realsense2_camera_msgs, srv, DeviceInfo_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, realsense2_camera_msgs, srv, DeviceInfo_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_realsense2_camera_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, realsense2_camera_msgs, srv, DeviceInfo)() {
  if (!realsense2_camera_msgs__srv__detail__device_info__rosidl_typesupport_introspection_c__DeviceInfo_service_type_support_handle.typesupport_identifier) {
    realsense2_camera_msgs__srv__detail__device_info__rosidl_typesupport_introspection_c__DeviceInfo_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)realsense2_camera_msgs__srv__detail__device_info__rosidl_typesupport_introspection_c__DeviceInfo_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, realsense2_camera_msgs, srv, DeviceInfo_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, realsense2_camera_msgs, srv, DeviceInfo_Response)()->data;
  }

  return &realsense2_camera_msgs__srv__detail__device_info__rosidl_typesupport_introspection_c__DeviceInfo_service_type_support_handle;
}
