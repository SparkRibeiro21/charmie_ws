// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from charmie_interfaces:msg/Keypoints.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/keypoints__rosidl_typesupport_fastrtps_cpp.hpp"
#include "charmie_interfaces/msg/detail/keypoints__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace charmie_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
cdr_serialize(
  const charmie_interfaces::msg::Keypoints & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: index_person
  cdr << ros_message.index_person;
  // Member: average_distance
  cdr << ros_message.average_distance;
  // Member: standard_deviation
  cdr << ros_message.standard_deviation;
  // Member: key_p0_x
  cdr << ros_message.key_p0_x;
  // Member: key_p0_y
  cdr << ros_message.key_p0_y;
  // Member: key_p1_x
  cdr << ros_message.key_p1_x;
  // Member: key_p1_y
  cdr << ros_message.key_p1_y;
  // Member: key_p2_x
  cdr << ros_message.key_p2_x;
  // Member: key_p2_y
  cdr << ros_message.key_p2_y;
  // Member: key_p3_x
  cdr << ros_message.key_p3_x;
  // Member: key_p3_y
  cdr << ros_message.key_p3_y;
  // Member: key_p4_x
  cdr << ros_message.key_p4_x;
  // Member: key_p4_y
  cdr << ros_message.key_p4_y;
  // Member: key_p5_x
  cdr << ros_message.key_p5_x;
  // Member: key_p5_y
  cdr << ros_message.key_p5_y;
  // Member: key_p6_x
  cdr << ros_message.key_p6_x;
  // Member: key_p6_y
  cdr << ros_message.key_p6_y;
  // Member: key_p7_x
  cdr << ros_message.key_p7_x;
  // Member: key_p7_y
  cdr << ros_message.key_p7_y;
  // Member: key_p8_x
  cdr << ros_message.key_p8_x;
  // Member: key_p8_y
  cdr << ros_message.key_p8_y;
  // Member: key_p9_x
  cdr << ros_message.key_p9_x;
  // Member: key_p9_y
  cdr << ros_message.key_p9_y;
  // Member: key_p10_x
  cdr << ros_message.key_p10_x;
  // Member: key_p10_y
  cdr << ros_message.key_p10_y;
  // Member: key_p11_x
  cdr << ros_message.key_p11_x;
  // Member: key_p11_y
  cdr << ros_message.key_p11_y;
  // Member: key_p12_x
  cdr << ros_message.key_p12_x;
  // Member: key_p12_y
  cdr << ros_message.key_p12_y;
  // Member: key_p13_x
  cdr << ros_message.key_p13_x;
  // Member: key_p13_y
  cdr << ros_message.key_p13_y;
  // Member: key_p14_x
  cdr << ros_message.key_p14_x;
  // Member: key_p14_y
  cdr << ros_message.key_p14_y;
  // Member: key_p15_x
  cdr << ros_message.key_p15_x;
  // Member: key_p15_y
  cdr << ros_message.key_p15_y;
  // Member: key_p16_x
  cdr << ros_message.key_p16_x;
  // Member: key_p16_y
  cdr << ros_message.key_p16_y;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  charmie_interfaces::msg::Keypoints & ros_message)
{
  // Member: index_person
  cdr >> ros_message.index_person;

  // Member: average_distance
  cdr >> ros_message.average_distance;

  // Member: standard_deviation
  cdr >> ros_message.standard_deviation;

  // Member: key_p0_x
  cdr >> ros_message.key_p0_x;

  // Member: key_p0_y
  cdr >> ros_message.key_p0_y;

  // Member: key_p1_x
  cdr >> ros_message.key_p1_x;

  // Member: key_p1_y
  cdr >> ros_message.key_p1_y;

  // Member: key_p2_x
  cdr >> ros_message.key_p2_x;

  // Member: key_p2_y
  cdr >> ros_message.key_p2_y;

  // Member: key_p3_x
  cdr >> ros_message.key_p3_x;

  // Member: key_p3_y
  cdr >> ros_message.key_p3_y;

  // Member: key_p4_x
  cdr >> ros_message.key_p4_x;

  // Member: key_p4_y
  cdr >> ros_message.key_p4_y;

  // Member: key_p5_x
  cdr >> ros_message.key_p5_x;

  // Member: key_p5_y
  cdr >> ros_message.key_p5_y;

  // Member: key_p6_x
  cdr >> ros_message.key_p6_x;

  // Member: key_p6_y
  cdr >> ros_message.key_p6_y;

  // Member: key_p7_x
  cdr >> ros_message.key_p7_x;

  // Member: key_p7_y
  cdr >> ros_message.key_p7_y;

  // Member: key_p8_x
  cdr >> ros_message.key_p8_x;

  // Member: key_p8_y
  cdr >> ros_message.key_p8_y;

  // Member: key_p9_x
  cdr >> ros_message.key_p9_x;

  // Member: key_p9_y
  cdr >> ros_message.key_p9_y;

  // Member: key_p10_x
  cdr >> ros_message.key_p10_x;

  // Member: key_p10_y
  cdr >> ros_message.key_p10_y;

  // Member: key_p11_x
  cdr >> ros_message.key_p11_x;

  // Member: key_p11_y
  cdr >> ros_message.key_p11_y;

  // Member: key_p12_x
  cdr >> ros_message.key_p12_x;

  // Member: key_p12_y
  cdr >> ros_message.key_p12_y;

  // Member: key_p13_x
  cdr >> ros_message.key_p13_x;

  // Member: key_p13_y
  cdr >> ros_message.key_p13_y;

  // Member: key_p14_x
  cdr >> ros_message.key_p14_x;

  // Member: key_p14_y
  cdr >> ros_message.key_p14_y;

  // Member: key_p15_x
  cdr >> ros_message.key_p15_x;

  // Member: key_p15_y
  cdr >> ros_message.key_p15_y;

  // Member: key_p16_x
  cdr >> ros_message.key_p16_x;

  // Member: key_p16_y
  cdr >> ros_message.key_p16_y;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
get_serialized_size(
  const charmie_interfaces::msg::Keypoints & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: index_person
  {
    size_t item_size = sizeof(ros_message.index_person);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: average_distance
  {
    size_t item_size = sizeof(ros_message.average_distance);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: standard_deviation
  {
    size_t item_size = sizeof(ros_message.standard_deviation);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p0_x
  {
    size_t item_size = sizeof(ros_message.key_p0_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p0_y
  {
    size_t item_size = sizeof(ros_message.key_p0_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p1_x
  {
    size_t item_size = sizeof(ros_message.key_p1_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p1_y
  {
    size_t item_size = sizeof(ros_message.key_p1_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p2_x
  {
    size_t item_size = sizeof(ros_message.key_p2_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p2_y
  {
    size_t item_size = sizeof(ros_message.key_p2_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p3_x
  {
    size_t item_size = sizeof(ros_message.key_p3_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p3_y
  {
    size_t item_size = sizeof(ros_message.key_p3_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p4_x
  {
    size_t item_size = sizeof(ros_message.key_p4_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p4_y
  {
    size_t item_size = sizeof(ros_message.key_p4_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p5_x
  {
    size_t item_size = sizeof(ros_message.key_p5_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p5_y
  {
    size_t item_size = sizeof(ros_message.key_p5_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p6_x
  {
    size_t item_size = sizeof(ros_message.key_p6_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p6_y
  {
    size_t item_size = sizeof(ros_message.key_p6_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p7_x
  {
    size_t item_size = sizeof(ros_message.key_p7_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p7_y
  {
    size_t item_size = sizeof(ros_message.key_p7_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p8_x
  {
    size_t item_size = sizeof(ros_message.key_p8_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p8_y
  {
    size_t item_size = sizeof(ros_message.key_p8_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p9_x
  {
    size_t item_size = sizeof(ros_message.key_p9_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p9_y
  {
    size_t item_size = sizeof(ros_message.key_p9_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p10_x
  {
    size_t item_size = sizeof(ros_message.key_p10_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p10_y
  {
    size_t item_size = sizeof(ros_message.key_p10_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p11_x
  {
    size_t item_size = sizeof(ros_message.key_p11_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p11_y
  {
    size_t item_size = sizeof(ros_message.key_p11_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p12_x
  {
    size_t item_size = sizeof(ros_message.key_p12_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p12_y
  {
    size_t item_size = sizeof(ros_message.key_p12_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p13_x
  {
    size_t item_size = sizeof(ros_message.key_p13_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p13_y
  {
    size_t item_size = sizeof(ros_message.key_p13_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p14_x
  {
    size_t item_size = sizeof(ros_message.key_p14_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p14_y
  {
    size_t item_size = sizeof(ros_message.key_p14_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p15_x
  {
    size_t item_size = sizeof(ros_message.key_p15_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p15_y
  {
    size_t item_size = sizeof(ros_message.key_p15_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p16_x
  {
    size_t item_size = sizeof(ros_message.key_p16_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: key_p16_y
  {
    size_t item_size = sizeof(ros_message.key_p16_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
max_serialized_size_Keypoints(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: index_person
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: average_distance
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: standard_deviation
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p0_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p0_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p1_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p1_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p2_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p2_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p3_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p3_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p4_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p4_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p5_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p5_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p6_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p6_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p7_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p7_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p8_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p8_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p9_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p9_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p10_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p10_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p11_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p11_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p12_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p12_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p13_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p13_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p14_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p14_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p15_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p15_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p16_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: key_p16_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _Keypoints__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const charmie_interfaces::msg::Keypoints *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Keypoints__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<charmie_interfaces::msg::Keypoints *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Keypoints__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const charmie_interfaces::msg::Keypoints *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Keypoints__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_Keypoints(full_bounded, 0);
}

static message_type_support_callbacks_t _Keypoints__callbacks = {
  "charmie_interfaces::msg",
  "Keypoints",
  _Keypoints__cdr_serialize,
  _Keypoints__cdr_deserialize,
  _Keypoints__get_serialized_size,
  _Keypoints__max_serialized_size
};

static rosidl_message_type_support_t _Keypoints__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Keypoints__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace charmie_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_charmie_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<charmie_interfaces::msg::Keypoints>()
{
  return &charmie_interfaces::msg::typesupport_fastrtps_cpp::_Keypoints__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, charmie_interfaces, msg, Keypoints)() {
  return &charmie_interfaces::msg::typesupport_fastrtps_cpp::_Keypoints__handle;
}

#ifdef __cplusplus
}
#endif
