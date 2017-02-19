option(BuildRos2ComponentExamples "BuildRos2ComponentExamples" OFF)
if(BuildRos2ComponentExamples)
include_directories(
    
    ${rclcpp_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
    ${rmw_implementation_INCLUDE_DIRS}
    ${ros2_components_msg_INCLUDE_DIRS}
    ${ros2_simple_logger_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS} 
    ${Qt5Network_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/ComponentFilter
    ${CMAKE_CURRENT_SOURCE_DIR}
)


file(GLOB COMPONENTS_SRC_FILES 
	simple_hardware_node.h
	simple_hardware_node.cpp
	example_entity.cpp
	example_entity.h
)
add_library(ros2_components_examples_lib
    ${COMPONENTS_SRC_FILES}
)
target_link_libraries(ros2_components_examples_lib
    ${rclcpp_LIBRARIES}
    ${rmw_LIBRARIES}
    ${rmw_implementation_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${builtin_interfaces_LIBRARIES}
    ${ros2_components_msg_LIBRARIES}
    ${ros2_simple_logger_LIBRARIES}
    -pthread
    Qt5::Core
    Qt5::Network

)

add_executable(dedicated_node dedicated_node_main.cpp)
add_executable(hardware_node hardware_node_main.cpp)


target_link_libraries(dedicated_node
    ros2_components_examples_lib
    ${rclcpp_LIBRARIES}
    ${rmw_LIBRARIES}
    ${rmw_implementation_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${builtin_interfaces_LIBRARIES}
    ${ros2_components_msg_LIBRARIES}
    ${ros2_simple_logger_LIBRARIES}
    -pthread
    Qt5::Core
    Qt5::Network

)

target_link_libraries(hardware_node
    ros2_components_examples_lib
    ${rclcpp_LIBRARIES}
    ${rmw_LIBRARIES}
    ${rmw_implementation_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${builtin_interfaces_LIBRARIES}
    ${ros2_components_msg_LIBRARIES}
    ${ros2_simple_logger_LIBRARIES}
    -pthread
    Qt5::Core
    Qt5::Network

)


install(TARGETS hardware_node DESTINATION bin)
install(TARGETS dedicated_node DESTINATION bin)
endif()
