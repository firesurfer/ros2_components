// generated from rosidl_generator_cpp/resource/srv__traits.hpp.template


#include "ros2_components_msg/srv/list_childs__struct.hpp"

namespace rosidl_generator_traits
{

#ifndef __ROSIDL_GENERATOR_CPP_TRAITS
#define __ROSIDL_GENERATOR_CPP_TRAITS

template<typename T>
struct has_fixed_size : std::false_type {};

#endif  /* __ROSIDL_GENERATOR_CPP_TRAITS */

#ifndef __ros2_components_msg__srv__list_childs__traits__hpp__
#define __ros2_components_msg__srv__list_childs__traits__hpp__
template <>
struct has_fixed_size<ros2_components_msg::srv::ListChilds> :
std::integral_constant<
  bool,
  has_fixed_size<ros2_components_msg::srv::ListChilds_Request>::value &&
  has_fixed_size<ros2_components_msg::srv::ListChilds_Response>::value> {};

#endif  // __ros2_components_msg__srv__list_childs__traits__hpp__
}  /* rosidl_generator_traits */
