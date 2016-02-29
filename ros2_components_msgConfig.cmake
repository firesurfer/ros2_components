if(_ros2_components_msg_CONFIG_INCLUDED)
  return()
endif()
set(_ros2_components_msg_CONFIG_INCLUDED TRUE)

set(ROS2_COMPONENTS_MSG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ros2_components_msg)
MESSAGE("ros2_components are are in: " ${ROS2_COMPONENTS_MSG_DIR})

file(GLOB LIB_FILES 
	${ROS2_COMPONENTS_MSG_DIR}/build/install/lib/*.a
	${ROS2_COMPONENTS_MSG_DIR}/build/install/lib/*.so 

)

set(ros2_components_msg_LIBRARIES ${LIB_FILES})

set(ros2_components_msg_INCLUDE_DIRS  
	${ROS2_COMPONENTS_MSG_DIR}/build/install/include


)
