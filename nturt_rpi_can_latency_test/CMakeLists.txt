cmake_minimum_required(VERSION 3.8)
project(nturt_rpi_can_latency_test)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# install python module for this package
ament_python_install_package(${PROJECT_NAME})

include_directories(
        include
)

# libraries
# library: nturt_rpi_can_latency_test
add_library(${PROJECT_NAME} STATIC
        src/rpi_can_latency_test.cpp
)
ament_target_dependencies(${PROJECT_NAME}
        ament_index_cpp
        can_msgs
        rclcpp
        std_msgs
)

# applications
# executable: nturt_rpi_can_latency_test_node
add_executable(${PROJECT_NAME}_node
        app/rpi_can_latency_test_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
)
ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
)

# tests
# executable: fake_socket_can_bridge_node
add_executable(fake_socket_can_bridge_node
        test/fake_socket_can_bridge_node.cpp
)
ament_target_dependencies(fake_socket_can_bridge_node
        can_msgs
        rclcpp
)

# install headers
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include/${PROJECT_NAME}
)

# install libraries
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

# install applications, test executables
install(TARGETS ${PROJECT_NAME}_node fake_socket_can_bridge_node
        DESTINATION lib/${PROJECT_NAME}
)

# install python scripts
install(PROGRAMS scripts/plot_test_result.py
        DESTINATION lib/${PROJECT_NAME})

# install launch scripts
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
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
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        ament_cmake
        ament_cmake_python
        ament_index
        can_msgs
        rclcpp
        rclpy
        std_msgs
)

ament_package()
