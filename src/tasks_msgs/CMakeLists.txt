cmake_minimum_required(VERSION 3.8)
project(tasks_msgs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# This command finds your .srv file and generates the code for it
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DrawShape.srv"
  DEPENDENCIES geometry_msgs
)

# This finalizes the package
ament_package()

