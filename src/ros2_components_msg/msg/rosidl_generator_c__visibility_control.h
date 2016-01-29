// generated from rosidl_generator_c/resource/rosidl_generator_c__visibility_control.h.in

#ifndef ros2_components_msg__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL__H_
#define ros2_components_msg__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL__H_

#if __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_C_EXPORT_ros2_components_msg \
      __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_C_IMPORT_ros2_components_msg \
      __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_C_EXPORT_ros2_components_msg \
      __declspec(dllexport)
    #define ROSIDL_GENERATOR_C_IMPORT_ros2_components_msg \
      __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_C_BUILDING_DLL_ros2_components_msg
    #define ROSIDL_GENERATOR_C_PUBLIC_ros2_components_msg \
      ROSIDL_GENERATOR_C_EXPORT_ros2_components_msg
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_ros2_components_msg \
      ROSIDL_GENERATOR_C_IMPORT_ros2_components_msg
  #endif
#else
  #define ROSIDL_GENERATOR_C_EXPORT_ros2_components_msg \
    __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_C_IMPORT_ros2_components_msg
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_C_PUBLIC_ros2_components_msg \
      __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_ros2_components_msg
  #endif
#endif

#if __cplusplus
}
#endif

#endif  // ros2_components_msg__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL__H_
