cmake_minimum_required(VERSION 3.8)
project(nturt_micro_ros_latency_test_host)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nturt_ros_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(
        include
)

# libraries
# library: nturt_micro_ros_latency_test_host
add_library(${PROJECT_NAME} SHARED
        src/micro_ros_latency_test_host.cpp
)
ament_target_dependencies(${PROJECT_NAME}
        nturt_ros_interface
        rclcpp
        rclcpp_components
)
# register to node components
# node component: MicroRosLatencyTestHost
rclcpp_components_register_nodes(nturt_micro_ros_latency_test_host "MicroRosLatencyTestHost")
set(node_plugins "${node_plugins}MicroRosLatencyTestHost;$<TARGET_FILE:nturt_micro_ros_latency_test_host>\n")

# applications
# executable: nturt_micro_ros_latency_test_host_node
add_executable(${PROJECT_NAME}_node
        app/micro_ros_latency_test_host_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
)
ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
)

# install headers
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)

# install libraries
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

# install applications
install(TARGETS ${PROJECT_NAME}_node
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
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        ament_cmake
        ament_index
        nturt_ros_interface
        rclcpp
)

ament_package()
