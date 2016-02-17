if(_ros2_components_CONFIG_INCLUDED)
  return()
endif()
set(_ros2_components_CONFIG_INCLUDED TRUE)

set(ROS2_COMPONENTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ros2_components)
MESSAGE("ros2_components are are in: " ${ROS2_COMPONENTS_DIR})

file(GLOB LIB_FILES 
	${ROS2_COMPONENTS_DIR}/build/*.a
	${ROS2_COMPONENTS_DIR}/build/*.so 

)

set(ros2_components_LIBRARIES ${LIB_FILES})

set(ros2_components_INCLUDE_DIRS  
	${ROS2_COMPONENTS_DIR}/src

)
