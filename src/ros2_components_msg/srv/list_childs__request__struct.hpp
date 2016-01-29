// generated from rosidl_generator_cpp/resource/msg__struct.hpp.template
// generated code does not contain a copyright notice

#ifndef ROS2_COMPONENTS_MSG__SRV__LIST_CHILDS__REQUEST__STRUCT_HPP_
#define ROS2_COMPONENTS_MSG__SRV__LIST_CHILDS__REQUEST__STRUCT_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

// include message dependencies

#ifndef _WIN32
# define DEPRECATED_ros2_components_msg_srv_ListChilds_Request __attribute__((deprecated))
#else
# define DEPRECATED_ros2_components_msg_srv_ListChilds_Request __declspec(deprecated)
#endif

namespace ros2_components_msg
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ListChilds_Request_
{
  using Type = ListChilds_Request_<ContainerAllocator>;

  ListChilds_Request_()
  {
  }
  explicit ListChilds_Request_(const ContainerAllocator & _alloc)
  {
    (void)_alloc;
  }

  // field types and members

  // setters for named parameter idiom

  // constants

  // pointer types
  using RawPtr =
    ros2_components_msg::srv::ListChilds_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_components_msg::srv::ListChilds_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_components_msg::srv::ListChilds_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_components_msg::srv::ListChilds_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
    ros2_components_msg::srv::ListChilds_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_components_msg::srv::ListChilds_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
    ros2_components_msg::srv::ListChilds_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_components_msg::srv::ListChilds_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_components_msg::srv::ListChilds_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_components_msg::srv::ListChilds_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED_ros2_components_msg_srv_ListChilds_Request
    std::shared_ptr<ros2_components_msg::srv::ListChilds_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED_ros2_components_msg_srv_ListChilds_Request
    std::shared_ptr<ros2_components_msg::srv::ListChilds_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ListChilds_Request_ & other) const
  {
    (void)other;
    return true;
  }
  bool operator!=(const ListChilds_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ListChilds_Request_

// alias to use template instance with default allocator
using ListChilds_Request =
    ros2_components_msg::srv::ListChilds_Request_<std::allocator<void>>;

// constants requiring out of line definition

}  // namespace srv

}  // namespace ros2_components_msg

#endif  // ROS2_COMPONENTS_MSG__SRV__LIST_CHILDS__REQUEST__STRUCT_HPP_
