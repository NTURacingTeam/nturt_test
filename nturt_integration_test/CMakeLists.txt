cmake_minimum_required(VERSION 3.8)
project(nturt_integration_test)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(FTXUI_ENABLE_INSTALL OFF)

add_subdirectory(FTXUI)

find_package(ament_cmake REQUIRED)
find_package(can_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nturt_can_config REQUIRED)
find_package(nturt_ros_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
        include
)

# libraries
# library: nturt_integration_test
add_library(${PROJECT_NAME} SHARED
        src/integration_test.cpp
        src/user_interface.cpp
)
target_link_libraries(${PROJECT_NAME}
        ftxui::component
        ftxui::dom
        ftxui::screen
)
ament_target_dependencies(${PROJECT_NAME}
        can_msgs
        nturt_can_config
        nturt_ros_interface
        rclcpp
        rclcpp_components
)
# register to node components
# node component: IntegrationTest
rclcpp_components_register_nodes(nturt_integration_test "IntegrationTest")
set(node_plugins "${node_plugins}IntegrationTest;$<TARGET_FILE:nturt_integration_test>\n")

# executables
# application: nturt_screen_controller_node
add_executable(${PROJECT_NAME}_node
        app/integration_test_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
)
ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
)

# test: fake_data_generator_node
add_executable(fake_data_generator_node
        test/fake_data_generator_node.cpp
)
ament_target_dependencies(fake_data_generator_node
        can_msgs
        geometry_msgs
        nturt_can_config
        rclcpp
        sensor_msgs
)

# test: user_interface_test
add_executable(user_interface_test
        test/user_interface_test.cpp
)
target_link_libraries(user_interface_test
        ${PROJECT_NAME}
)

# install headers
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)

# install applications executables
install(TARGETS ${PROJECT_NAME}_node fake_data_generator_node user_interface_test
        DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        # the following line skips the linter which checks for copyrights
        # comment the line when a copyright and license is added to all source files
        set(ament_cmake_copyright_FOUND TRUE)
        # the following line skips cpplint (only works in a git repo)
        # comment the line when this package is in a git repo and when
        # a copyright and license is added to all source files
        set(ament_cmake_cpplint_FOUND TRUE)
        ament_lint_auto_find_test_dependencies()
endif()

# export package dependencies
ament_export_include_directories(
        include
)
ament_export_dependencies(
        ament_cmake
        can_msgs
        nturt_can_config
        nturt_ros_interface
        rclcpp
)

ament_package()
