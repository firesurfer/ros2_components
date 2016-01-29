// generated from rosidl_generator_c/resource/msg__functions.h.template
// generated code does not contain a copyright notice

#ifndef ros2_components_msg__srv__list_childs__response__functions_h_
#define ros2_components_msg__srv__list_childs__response__functions_h_

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_generator_c/visibility_control.h"
#include "ros2_components_msg/msg/rosidl_generator_c__visibility_control.h"

#include "ros2_components_msg/srv/list_childs__response__struct.h"

/// Initialize ros2_components_msg/ListChilds_Response message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(ros2_components_msg__srv__ListChilds_Response)) before
 * or use ros2_components_msg__srv__ListChilds_Response__create() to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_components_msg
bool
ros2_components_msg__srv__ListChilds_Response__init(ros2_components_msg__srv__ListChilds_Response * msg);

/// Finalize ros2_components_msg/ListChilds_Response message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_components_msg
void
ros2_components_msg__srv__ListChilds_Response__fini(ros2_components_msg__srv__ListChilds_Response * msg);

/// Create ros2_components_msg/ListChilds_Response message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls ros2_components_msg__srv__ListChilds_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_components_msg
ros2_components_msg__srv__ListChilds_Response *
ros2_components_msg__srv__ListChilds_Response__create();

/// Destroy ros2_components_msg/ListChilds_Response message.
/**
 * It calls ros2_components_msg__srv__ListChilds_Response__fini() and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_components_msg
void
ros2_components_msg__srv__ListChilds_Response__destroy(ros2_components_msg__srv__ListChilds_Response * msg);


/// Initialize array of ros2_components_msg/ListChilds_Response messages.
/**
 * It allocates the memory for the number of elements and
 * calls ros2_components_msg__srv__ListChilds_Response__init() for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_components_msg
bool
ros2_components_msg__srv__ListChilds_Response__Array__init(ros2_components_msg__srv__ListChilds_Response__Array * array, size_t size);

/// Finalize array of ros2_components_msg/ListChilds_Response messages.
/**
 * It calls ros2_components_msg__srv__ListChilds_Response__fini() for each element of the array and
 * frees the memory for the number of elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_components_msg
void
ros2_components_msg__srv__ListChilds_Response__Array__fini(ros2_components_msg__srv__ListChilds_Response__Array * array);

/// Create array of ros2_components_msg/ListChilds_Response messages.
/**
 * It allocates the memory for the array and
 * calls ros2_components_msg__srv__ListChilds_Response__Array__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_components_msg
ros2_components_msg__srv__ListChilds_Response__Array *
ros2_components_msg__srv__ListChilds_Response__Array__create(size_t size);

/// Destroy array of ros2_components_msg/ListChilds_Response messages.
/**
 * It calls ros2_components_msg__srv__ListChilds_Response__Array__fini() on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2_components_msg
void
ros2_components_msg__srv__ListChilds_Response__Array__destroy(ros2_components_msg__srv__ListChilds_Response__Array * array);

#endif  // ros2_components_msg__srv__list_childs__response__functions_h_
