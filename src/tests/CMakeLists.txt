


include_directories(
    ${CMAKE_SOURCE_DIR}/src/ros2_components
    ${CMAKE_SOURCE_DIR}/src
    ${rclcpp_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
    ${rmw_implementation_INCLUDE_DIRS}
    ${ros2_components_msg_INCLUDE_DIRS}
    ${ros2_simple_logger_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS} 
    ${Qt5Network_INCLUDE_DIRS}
)

add_executable(ros2_components_create_robot_test create_robot.cpp)
target_link_libraries(ros2_components_create_robot_test
	ros2_components
 	${rclcpp_LIBRARIES}
    	${rmw_LIBRARIES}
    	${rmw_implementation_LIBRARIES}
    	${ros2_components_msg_LIBRARIES}
    	${ros2_simple_logger_LIBRARIES}
    	-pthread
    	Qt5::Core
    	Qt5::Network
)
install(TARGETS ros2_components_create_robot_test DESTINATION tests)
