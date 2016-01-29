// generated from rosidl_generator_c/resource/msg__struct.h.template
// generated code does not contain a copyright notice

#ifndef ROS2_COMPONENTS_MSG__SRV__LIST_CHILDS__RESPONSE__STRUCT_H_
#define ROS2_COMPONENTS_MSG__SRV__LIST_CHILDS__RESPONSE__STRUCT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// include message dependencies
// childids
#include "rosidl_generator_c/primitives_array.h"
// childtypes
#include "rosidl_generator_c/string.h"

/// Struct of message ros2_components_msg/ListChilds_Response
typedef struct ros2_components_msg__srv__ListChilds_Response
{
  int64_t listsize;
  rosidl_generator_c__int64__Array childids;
  rosidl_generator_c__String__Array childtypes;
} ros2_components_msg__srv__ListChilds_Response;

/// Struct for an array of messages
typedef struct ros2_components_msg__srv__ListChilds_Response__Array
{
  ros2_components_msg__srv__ListChilds_Response * data;
  size_t size;
  size_t capacity;
} ros2_components_msg__srv__ListChilds_Response__Array;

#endif  // ROS2_COMPONENTS_MSG__SRV__LIST_CHILDS__RESPONSE__STRUCT_H_
