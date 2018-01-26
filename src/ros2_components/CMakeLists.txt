include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${ros2_components_msg_INCLUDE_DIRS}
    ${ros2_simple_logger_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS} 
    ${Qt5Network_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/CLI
    ${CMAKE_CURRENT_SOURCE_DIR}/Exceptions
    ${CMAKE_CURRENT_SOURCE_DIR}/GUI
    ${CMAKE_CURRENT_SOURCE_DIR}
)


file(GLOB COMPONENTS_SRC_FILES 
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c*"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h*"

    "${CMAKE_CURRENT_SOURCE_DIR}/CLI/*.c*"
    "${CMAKE_CURRENT_SOURCE_DIR}/CLI/*.h*"

    "${CMAKE_CURRENT_SOURCE_DIR}/GUI/*.c*"
    "${CMAKE_CURRENT_SOURCE_DIR}/GUI/*.h*"
    "${CMAKE_CURRENT_SOURCE_DIR}/GUI/*.ui*"
	
    "${CMAKE_CURRENT_SOURCE_DIR}/Exceptions/*.c*"
    "${CMAKE_CURRENT_SOURCE_DIR}/Exceptions/*.h*"
)
add_library(ros2_components
    ${COMPONENTS_SRC_FILES}
)
ament_target_dependencies(ros2_components
    ros2_simple_logger
    ros2_components_msgs)
target_link_libraries(ros2_components 
    Threads::Threads
    Qt5::Core
    Qt5::Network
    Qt5::Widgets

)
install(TARGETS ros2_components DESTINATION lib)

