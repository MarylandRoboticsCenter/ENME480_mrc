cmake_minimum_required(VERSION 3.8)
project(ur3e_mrc_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ur3e_mrc_msgs REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# Install Python execs
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/ur3e_mrc_enme480_topics.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
