// generated from rosidl_generator_cpp/resource/msg__struct.hpp.template
// generated code does not contain a copyright notice

#ifndef ROS2_COMPONENTS_MSG__SRV__LIST_CHILDS__RESPONSE__STRUCT_HPP_
#define ROS2_COMPONENTS_MSG__SRV__LIST_CHILDS__RESPONSE__STRUCT_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

// include message dependencies

#ifndef _WIN32
# define DEPRECATED_ros2_components_msg_srv_ListChilds_Response __attribute__((deprecated))
#else
# define DEPRECATED_ros2_components_msg_srv_ListChilds_Response __declspec(deprecated)
#endif

namespace ros2_components_msg
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ListChilds_Response_
{
  using Type = ListChilds_Response_<ContainerAllocator>;

  ListChilds_Response_()
  {
  }
  explicit ListChilds_Response_(const ContainerAllocator & _alloc)
  : childids(_alloc),
    childtypes(_alloc)
  {
  }

  // field types and members
  using _childids_type =
      std::vector<int64_t, typename ContainerAllocator::template rebind<int64_t>::other>;
  _childids_type childids;
  using _childtypes_type =
      std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _childtypes_type childtypes;

  // setters for named parameter idiom
  Type * set__childids(
    const std::vector<int64_t, typename ContainerAllocator::template rebind<int64_t>::other> & _arg)
  {
    this->childids = _arg;
    return this;
  }
  Type * set__childtypes(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->childtypes = _arg;
    return this;
  }

  // constants

  // pointer types
  using RawPtr =
    ros2_components_msg::srv::ListChilds_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_components_msg::srv::ListChilds_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_components_msg::srv::ListChilds_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_components_msg::srv::ListChilds_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
    ros2_components_msg::srv::ListChilds_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_components_msg::srv::ListChilds_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
    ros2_components_msg::srv::ListChilds_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_components_msg::srv::ListChilds_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_components_msg::srv::ListChilds_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_components_msg::srv::ListChilds_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED_ros2_components_msg_srv_ListChilds_Response
    std::shared_ptr<ros2_components_msg::srv::ListChilds_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED_ros2_components_msg_srv_ListChilds_Response
    std::shared_ptr<ros2_components_msg::srv::ListChilds_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ListChilds_Response_ & other) const
  {
    if (this->childids != other.childids) {
      return false;
    }
    if (this->childtypes != other.childtypes) {
      return false;
    }
    return true;
  }
  bool operator!=(const ListChilds_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ListChilds_Response_

// alias to use template instance with default allocator
using ListChilds_Response =
    ros2_components_msg::srv::ListChilds_Response_<std::allocator<void>>;

// constants requiring out of line definition

}  // namespace srv

}  // namespace ros2_components_msg

#endif  // ROS2_COMPONENTS_MSG__SRV__LIST_CHILDS__RESPONSE__STRUCT_HPP_
