find_package("std_msgs" REQUIRED)

enable_testing()
find_package(GTest)

if(NOT ${GTEST_FOUND})
    MESSAGE("Did not find googletest - not building tests")
    return()
endif()

include_directories(
    ${CMAKE_SOURCE_DIR}/src/ros2_components
    ${CMAKE_SOURCE_DIR}/src
    ${rclcpp_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
    ${rmw_implementation_INCLUDE_DIRS}
    ${builtin_interfaces_INCLUDE_DIRS}
    ${ros2_components_msg_INCLUDE_DIRS}
    ${ros2_simple_logger_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Network_INCLUDE_DIRS}

    ${GTEST_INCLUDE_DIRS}
    )

add_executable(create_robot_test create_robot.cpp)
add_executable(create_robot_and_rebuild_test create_and_rebuild_robot.cpp)
add_executable(check_advertisements_test check_advertisements.cpp)
add_executable(static_class_test static_class_shared_ptr_test.cpp static_class_shared_ptr_test_exec.cpp)

target_link_libraries(create_robot_test
    ros2_components
    ${GTEST_BOTH_LIBRARIES}
    )
target_link_libraries(create_robot_and_rebuild_test
    ${GTEST_BOTH_LIBRARIES}
    ros2_components
    )
target_link_libraries(check_advertisements_test
    ${GTEST_BOTH_LIBRARIES}
    ros2_components
    )
target_link_libraries(static_class_test
    ${std_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${rmw_LIBRARIES}
    ${rmw_implementation_LIBRARIES}
    ${builtin_interfaces_LIBRARIES}
    -pthread
    )

add_test(AllTestsInCreateRobotTest create_robot_test)
add_test(AllTestsInCreateRobotAndRebuildTest create_robot_and_rebuild_test)
add_test(AllTestsInFCheckAdvertisementsTest check_advertisements_test)

install(TARGETS create_robot_test DESTINATION tests/ros2_components)
install(TARGETS create_robot_and_rebuild_test DESTINATION tests/ros2_components)

install(TARGETS check_advertisements_test DESTINATION tests/ros2_components)
install(TARGETS static_class_test DESTINATION tests/ros2_components)
