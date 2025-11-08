cmake_minimum_required(VERSION 3.8)
project(orbital_mechanics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(mechanics_node src/mechanics_node.cpp)
ament_target_dependencies(mechanics_node rclcpp std_msgs geometry_msgs visualization_msgs)

#ament_python_install_package(${PROJECT_NAME})

#install(PROGRAMS
#  # Progam
#  DESTINATION lib/${PROJECT_NAME}
#)

install(TARGETS
  mechanics_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
