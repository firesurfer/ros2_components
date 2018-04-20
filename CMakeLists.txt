cmake_minimum_required(VERSION 3.5)

project(ros2_components)

set (CMAKE_CXX_STANDARD 14)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -fPIC")
endif()

IF (NOT DEFINED BUILD_VERSION)
    SET(BUILD_VERSION "not set")
ENDIF()


option(BUILD_TIME_FROM_CMAKE "Set the build time via cmake. Might force rebuilds every build!" OFF)

if(BUILD_TIME_FROM_CMAKE)
string(TIMESTAMP BUILD_TIME "%H:%M:%S %d.%m.%Y" )
    ADD_DEFINITIONS(-DBUILD_VERSION="${BUILD_VERSION}" -DBUILD_TIME="${BUILD_TIME}")
else()
    ADD_DEFINITIONS(-DBUILD_VERSION="${BUILD_VERSION}")
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(ros2_components_msg REQUIRED)
find_package(ros2_simple_logger REQUIRED)
find_package(rmw_fastrtps_cpp REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/ros2_components)

file(GLOB INCLUDE_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ros2_components/*.h*"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ros2_components/ComponentFilter/*.h*"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ros2_components/CLI/*.h*"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ros2_components/GUI/*.h*"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ros2_components/Exceptions/*.h*"
)

install(FILES ${INCLUDE_FILES} DESTINATION include/ros2_components)

ament_export_include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Network_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${ros2_simple_logger_INCLUDE_DIRS}
    ${ros2_components_msg_INCLUDE_DIRS}
)

ament_export_libraries(ros2_components)
ament_export_dependencies(Qt5Core Qt5Network Qt5Widgets ros2_components_msg ros2_simple_logger ament_cmake builtin_interfaces rmw rmw_implementation_cmake std_msgs Threads rmw_fastrtps_cpp)

ament_package()
