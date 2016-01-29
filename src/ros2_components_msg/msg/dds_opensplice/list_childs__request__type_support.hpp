// generated from rosidl_typesupport_opensplice_cpp/resource/msg__type_support.hpp.template

#ifndef __ros2_components_msg__srv__dds_opensplice__list_childs__request__type_support__hpp__
#define __ros2_components_msg__srv__dds_opensplice__list_childs__request__type_support__hpp__

#include "ros2_components_msg/srv/list_childs__request__struct.hpp"
#include "ros2_components_msg/srv/dds_opensplice/ccpp_ListChilds_Request_.h"
#include "ros2_components_msg/msg/dds_opensplice/visibility_control.h"

namespace DDS
{
class DomainParticipant;
class DataReader;
class DataWriter;
}  // namespace DDS

namespace ros2_components_msg
{

namespace srv
{

namespace typesupport_opensplice_cpp
{

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ros2_components_msg
extern void register_type__ListChilds_Request(
  DDS::DomainParticipant * participant,
  const char * type_name);

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ros2_components_msg
extern void convert_ros_message_to_dds(
  const ros2_components_msg::srv::ListChilds_Request& ros_message,
  ros2_components_msg::srv::dds_::ListChilds_Request_& dds_message);

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ros2_components_msg
extern void publish__ListChilds_Request(
  DDS::DataWriter * topic_writer,
  const void * untyped_ros_message);

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ros2_components_msg
extern void convert_dds_message_to_ros(
  const ros2_components_msg::srv::dds_::ListChilds_Request_& dds_message,
  ros2_components_msg::srv::ListChilds_Request& ros_message);

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ros2_components_msg
extern bool take__ListChilds_Request(
  DDS::DataReader * topic_reader,
  bool ignore_local_publications,
  void * untyped_ros_message,
  bool * taken);

}  // namespace typesupport_opensplice_cpp

}  // namespace srv

}  // namespace ros2_components_msg

#endif  // __ros2_components_msg__srv__dds_opensplice__list_childs__request__type_support__hpp__
