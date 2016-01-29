cmake_minimum_required(VERSION 2.8.3)

project(ros2_components_msg)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)

find_package(builtin_interfaces REQUIRED)

find_package(rosidl_default_generators REQUIRED)

file(GLOB SERVICE_FILES
    "srv/*.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  #${MESSAGE_FILES}
  ${SERVICE_FILES}
  DEPENDENCIES builtin_interfaces
)

#target_link_libraries(${PROJECT_NAME}_msgs)
project(ros2_components)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

macro(targets)
	if(NOT "${target_suffix} " STREQUAL " ")
	  get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
	endif()

	include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src
	)
	
	file(GLOB SRC_FILES
	    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c*"
	    
	)
	add_library(ros2_components${target_suffix}
		${SRC_FILES}
	)

	ament_target_dependencies(ros2_components${target_suffix}
        	"rclcpp"
		"geometry_msgs"
		"tf2"
		"tf2_msgs"
		"tf2_ros"

	)

    	ament_export_libraries(ros2_components${target_suffix}
)

	install(TARGETS ros2_components${target_suffix}
                DESTINATION lib)


endmacro()

file(GLOB INCLUDE_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/*.h*"
)
message(STATUS "FILES TO INSTALL (ROS2_COMPONENTS)")
message(STATUS ${INCLUDE_FILES})
install(FILES ${INCLUDE_FILES} DESTINATION include/ros2_components) 



call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

ament_package()
