cmake_minimum_required(VERSION 3.5)


project(ros2_components)


if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra ")
endif()


find_package(ament_cmake REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(ros2_components_msg REQUIRED)
find_package(ros2_simple_logger REQUIRED)


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/ros2_components)
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/tests)

file(GLOB INCLUDE_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/ros2_components/*.h*"
)

install(FILES ${INCLUDE_FILES} DESTINATION include/ros2_components)



ament_export_include_directories(include)
ament_export_libraries(ros2_components)

ament_package()
