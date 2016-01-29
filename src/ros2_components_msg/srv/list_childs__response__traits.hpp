// generated from rosidl_generator_cpp/resource/msg__traits.hpp.template
// generated code does not contain a copyright notice

#ifndef ROS2_COMPONENTS_MSG__SRV__LIST_CHILDS__RESPONSE__TRAITS_HPP_
#define ROS2_COMPONENTS_MSG__SRV__LIST_CHILDS__RESPONSE__TRAITS_HPP_

#include <stdint.h>

#include <array>
#include <string>
#include <vector>

namespace rosidl_generator_traits
{

#ifndef __ROSIDL_GENERATOR_CPP_TRAITS
#define __ROSIDL_GENERATOR_CPP_TRAITS

template<typename T>
struct has_fixed_size : std::false_type {};

#endif  // __ROSIDL_GENERATOR_CPP_TRAITS

#include "ros2_components_msg/srv/list_childs__response__struct.hpp"


template<>
struct has_fixed_size<ros2_components_msg::srv::ListChilds_Response> :
std::integral_constant<bool, false> {};

}  // namespace rosidl_generator_traits

#endif  // ROS2_COMPONENTS_MSG__SRV__LIST_CHILDS__RESPONSE__TRAITS_HPP_
