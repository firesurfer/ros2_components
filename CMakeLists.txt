cmake_minimum_required(VERSION 2.8.3)


project(ros2_components)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()
#Uncomment this line if you want to build locally without ament
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
find_package(ament_cmake REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(ros2_components_msg REQUIRED)
#find_package(tf2 REQUIRED)
#find_package(tf2_ros REQUIRED)
#find_package(tf2_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ros2_components
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${rclcpp_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
    ${rmw_implementation_INCLUDE_DIRS}
 ${ros2_components_msg_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
)
file(GLOB SRC_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ros2_components/*.c*"
)
add_library(ros2_components${target_suffix}
    ${SRC_FILES}
)
#qt5_use_modules(ros2_components Core)

target_link_libraries(ros2_components
    ${rclcpp_LIBRARIES}
    ${rmw_LIBRARIES}
    ${rmw_implementation_LIBRARIES}
${ros2_components_msg_LIBRARIES}
    -pthread
    Qt5::Core

)

file(GLOB INCLUDE_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/ros2_components/*.h*"
)
message(STATUS "FILES TO INSTALL (ROS2_COMPONENTS)")
message(STATUS ${INCLUDE_FILES})
install(FILES ${INCLUDE_FILES} DESTINATION include/ros2_components) 
install(TARGETS ros2_components DESTINATION lib)


ament_export_include_directories(include)
ament_export_libraries(ros2_components)

ament_package()
