// generated from rosidl_typesupport_opensplice_cpp/resource/visibility_control.h.in

#ifndef __ros2_components_msg__msg__dds_opensplice__visibility_control__h__
#define __ros2_components_msg__msg__dds_opensplice__visibility_control__h__

#if __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_ros2_components_msg \
      __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_IMPORT_ros2_components_msg \
      __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_ros2_components_msg \
      __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_IMPORT_ros2_components_msg \
      __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_BUILDING_DLL_ros2_components_msg
    #define ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ros2_components_msg \
      ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_ros2_components_msg
  #else
    #define ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ros2_components_msg \
      ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_IMPORT_ros2_components_msg
  #endif
#else
  #define ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_ros2_components_msg
  #define ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ros2_components_msg
#endif

#if __cplusplus
}
#endif

#endif  // __ros2_components_msg__msg__dds_opensplice__visibility_control__h__
